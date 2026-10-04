"""Local command channel between the CLI and the supervisor (JSON lines on a Unix socket).

The socket lives in a directory only the owning user can open. Laptops reach it through
SSH by running the CLI on the Jetson; no control port is exposed on the network.
"""

import json
import os
from pathlib import Path
import socket
import socketserver
import threading
import time

COMMANDS = ("status", "map-start", "race-start", "stop", "heartbeat")


def default_socket_path():
    """Per-user socket path, overridable with AVLITE_ROBORACER_SOCKET."""
    if os.environ.get("AVLITE_ROBORACER_SOCKET"):
        return Path(os.environ["AVLITE_ROBORACER_SOCKET"])
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    base = Path(runtime) if runtime else Path("/tmp") / f"avlite-roboracer-{os.getuid()}"
    return base / "avlite-roboracer.sock"


def dispatch(supervisor, request, now, status_extra=None):
    """Apply one decoded request to the supervisor and return the JSON-ready reply."""
    if not isinstance(request, dict) or request.get("command") not in COMMANDS:
        return {"ok": False, "message": f"unknown command; use one of {', '.join(COMMANDS)}"}
    command = request["command"]
    session = request.get("session")
    if command == "status":
        return {"ok": True, **supervisor.status(now, status_extra() if status_extra else None)}
    if command == "map-start":
        return supervisor.map_start(session, now)
    if command == "race-start":
        return supervisor.race_start(session, now)
    if command == "heartbeat":
        return supervisor.heartbeat(session, now)
    return supervisor.stop(now, request.get("reason") or "stop command")


class CommandServer:
    """Serve requests on a Unix socket in a background thread.

    handler(request) must be thread-safe; the runtime wraps the supervisor in a lock.
    """

    def __init__(self, path, handler):
        import fcntl  # Jetson/Linux: process death also releases this exclusive lock.
        self.path = Path(path)
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        self._lock_file = self.path.with_suffix(".lock").open("a")
        try:
            fcntl.flock(self._lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._lock_file.close()
            raise RuntimeError(f"supervisor already owns {self.path}") from exc
        if self.path.exists():
            self.path.unlink()
        outer = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                for line in self.rfile:
                    try:
                        reply = outer.handler(json.loads(line))
                    except (ValueError, TypeError) as exc:
                        reply = {"ok": False, "message": f"invalid request: {exc}"}
                    self.wfile.write((json.dumps(reply, allow_nan=False) + "\n").encode())
                    self.wfile.flush()

        self.handler = handler
        self.server = socketserver.ThreadingUnixStreamServer(str(self.path), Handler)
        self.server.daemon_threads = True
        os.chmod(self.path, 0o600)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        if self.path.exists():
            self.path.unlink()
        self._lock_file.close()


class Client:
    """Persistent connection used by the CLI (one request, or a heartbeat stream)."""

    def __init__(self, path=None, timeout_s=2.0):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout_s)
        try:
            self.sock.connect(str(path or default_socket_path()))
        except OSError as exc:
            self.sock.close()
            raise ConnectionError(f"supervisor is not running ({exc})") from exc
        self.stream = self.sock.makefile("rwb")

    def request(self, command, **fields):
        self.stream.write((json.dumps({"command": command, **fields,
                                       "sent_unix_s": time.time()}) + "\n").encode())
        self.stream.flush()
        line = self.stream.readline()
        if not line:
            raise ConnectionError("supervisor closed the connection")
        return json.loads(line)

    def close(self):
        try:
            self.stream.close()
        finally:
            self.sock.close()
