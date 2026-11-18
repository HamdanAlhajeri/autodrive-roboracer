"""Live pose view: where the car thinks it is, next to where the simulator says it is.

The ``PoseComparisonView`` AVLite task (``plugin/pose_view.py``) publishes one snapshot per
control cycle into a ``PoseHub``. ``PoseViewServer`` serves a single web page plus a
server-sent-event stream of those snapshots, so a browser on the host shows both cars on the
track map in real time. This module has no AVLite or ROS imports: it is plain data and
standard-library HTTP.

Nothing here reaches back into the car. The true pose is the same read-only diagnostic as
``localization_error_m``; the view only displays it.

Enable it in ``config/avlite.yaml``::

    visualization:
      pose_view: true
      port: 8765
      error_alert_m: 0.5
"""

from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import math
from pathlib import Path
import sys
import threading

log = logging.getLogger(__name__)

PAGE = Path(__file__).with_name("pose_view.html")
_KEYS = {"pose_view", "port", "error_alert_m"}
MAX_HITS = 3000  # map points sent to the page; more adds weight, not information


@dataclass(frozen=True)
class PoseViewConfig:
    port: int = 8765
    error_alert_m: float = 0.5   # the page turns the error red beyond this distance
    host: str = "0.0.0.0"        # inside the container; Docker publishes it on 127.0.0.1 only


def pose_view_config(config):
    """Return a PoseViewConfig when ``visualization.pose_view`` is true, otherwise None.

    Reject unknown keys and bad values before ROS starts, so a typo cannot silently disable
    the view or bind a surprising port.
    """
    block = config.get("visualization") or {}
    if not isinstance(block, dict):
        raise ValueError("visualization must be a mapping")
    unknown = set(block) - _KEYS
    if unknown:
        raise ValueError("Unknown visualization setting: " + ", ".join(sorted(unknown)))
    enabled = block.get("pose_view", False)
    if not isinstance(enabled, bool):
        raise ValueError("visualization.pose_view must be true or false")
    port = block.get("port", PoseViewConfig.port)
    if isinstance(port, bool) or not isinstance(port, int) or not 1024 <= port <= 65535:
        raise ValueError("visualization.port must be an integer from 1024 to 65535")
    alert = block.get("error_alert_m", PoseViewConfig.error_alert_m)
    if (isinstance(alert, bool) or not isinstance(alert, (int, float))
            or not math.isfinite(alert) or alert <= 0):
        raise ValueError("visualization.error_alert_m must be a positive number")
    return PoseViewConfig(port=port, error_alert_m=float(alert)) if enabled else None


def _finite_pose(pose):
    """Return (x, y, yaw) as floats, or None unless all three are finite numbers."""
    if pose is None or len(pose) < 3:
        return None
    values = tuple(float(v) for v in pose[:3])
    return values if all(math.isfinite(v) for v in values) else None


def build_snapshot(t, estimate, truth=None, alignment=None, hits=None, speed=None):
    """Return one JSON-ready message, or None when the estimate itself is unusable.

    ``estimate`` and ``truth`` are (x, y, yaw) in the world frame. ``truth`` is None when the
    simulator pose is not available (for example ground-truth localization, where the
    estimate already is the truth). The yaw error is signed, estimate minus truth, in radians.
    """
    est = _finite_pose(estimate)
    if est is None:
        return None
    snapshot = {"t": round(float(t), 3), "est": [round(v, 4) for v in est]}
    real = _finite_pose(truth)
    if real is not None:
        yaw = math.atan2(math.sin(est[2] - real[2]), math.cos(est[2] - real[2]))
        snapshot.update(truth=[round(v, 4) for v in real],
                        err_m=round(math.hypot(est[0] - real[0], est[1] - real[1]), 4),
                        yaw_err_rad=round(yaw, 4))
    else:
        snapshot.update(truth=None, err_m=None, yaw_err_rad=None)
    for key, value in (("alignment", alignment), ("hits", hits), ("speed", speed)):
        snapshot[key] = (round(float(value), 4)
                         if value is not None and math.isfinite(float(value)) else None)
    return snapshot


def _rounded(points, limit=None):
    points = [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in points]
    if limit and len(points) > limit:
        step = math.ceil(len(points) / limit)
        points = points[::step]
    return points


def geometry_from_map(race_map, path_points=None):
    """Static track layout for the page: boundaries, optional racing line and LiDAR map.

    The recorded LiDAR returns are read from the map file when it has them; a file that
    cannot be read just leaves them out.
    """
    geometry = {"left": _rounded(race_map.left_bound), "right": _rounded(race_map.right_bound),
                "path": _rounded(path_points) if path_points is not None else [], "hits": []}
    try:
        data = json.loads(Path(race_map.source_path).read_text(encoding="utf-8"))
        geometry["hits"] = _rounded(data.get("observed_hits") or [], MAX_HITS)
    except (OSError, ValueError, AttributeError, TypeError):
        pass
    return geometry


class PoseHub:
    """The latest snapshot plus a wake-up for every connected page.

    The control loop calls ``publish`` and returns at once; each browser connection waits
    in its own server thread. A slow or stalled browser never delays the car.
    """

    def __init__(self, geometry=None, error_alert_m=0.5):
        self.geometry = geometry or {"left": [], "right": [], "path": [], "hits": []}
        self.error_alert_m = error_alert_m
        self._cond = threading.Condition()
        self._seq = 0
        self._epoch = 0
        self._latest = None
        self.closed = False

    @property
    def epoch(self):
        with self._cond:
            return self._epoch

    def publish(self, snapshot):
        with self._cond:
            self._seq += 1
            self._latest = snapshot
            self._cond.notify_all()

    def reset(self):
        """Tell every page to clear its trails (the simulator was reset)."""
        with self._cond:
            self._epoch += 1
            self._latest = None
            self._cond.notify_all()

    def close(self):
        with self._cond:
            self.closed = True
            self._cond.notify_all()

    def wait(self, seq, epoch, timeout):
        """Block until something newer than (seq, epoch) exists; return (seq, epoch, snapshot).

        The snapshot is None when the wait timed out or only a reset happened.
        """
        with self._cond:
            self._cond.wait_for(
                lambda: self.closed or self._seq != seq or self._epoch != epoch, timeout)
            fresh = self._latest if self._seq != seq else None
            return self._seq, self._epoch, fresh


def _handler(hub):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):  # a 20 Hz stream would flood the controller log
            pass

        def _send(self, status, body, content_type):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            elif path == "/geometry":
                body = json.dumps({**hub.geometry, "error_alert_m": hub.error_alert_m})
                self._send(200, body.encode(), "application/json")
            elif path == "/events":
                self._stream()
            else:
                self._send(404, b"not found", "text/plain")

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            seq, epoch = 0, hub.epoch
            try:
                while not hub.closed:
                    seq, new_epoch, snapshot = hub.wait(seq, epoch, 10.0)
                    if new_epoch != epoch:
                        epoch = new_epoch
                        self.wfile.write(b"event: reset\ndata: {}\n\n")
                    if snapshot is not None:
                        self.wfile.write(b"data: " + json.dumps(snapshot).encode() + b"\n\n")
                    elif new_epoch == epoch:
                        self.wfile.write(b": keepalive\n\n")  # also notices closed sockets
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionError, OSError):
                pass

    return Handler


class _QuietServer(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        """Browsers drop connections all the time; only log what is not a dropped connection."""
        if not isinstance(sys.exc_info()[1], ConnectionError):
            super().handle_error(request, client_address)


class PoseViewServer:
    """Serve the page and event stream from daemon threads; ``close`` stops them."""

    def __init__(self, hub, config):
        self.hub, self.config = hub, config
        self._server = None

    @property
    def port(self):
        return self._server.server_address[1] if self._server else None

    def start(self):
        """Bind and serve. Raise OSError if the port is taken; the caller decides what then."""
        server = _QuietServer((self.config.host, self.config.port), _handler(self.hub))
        self._server = server
        threading.Thread(target=server.serve_forever, name="pose-view", daemon=True).start()
        log.info("Pose view: open http://localhost:%d", self.port)

    def close(self):
        self.hub.close()
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
