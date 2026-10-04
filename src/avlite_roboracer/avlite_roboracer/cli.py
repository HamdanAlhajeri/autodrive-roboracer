"""Operator commands for the Jetson supervisor; run locally or through SSH from the laptop.

    avlite-roboracer status [--json]
    avlite-roboracer map-start [--session S] [--heartbeat-stdin]
    avlite-roboracer race-start [--session S] [--heartbeat-stdin]
    avlite-roboracer stop

map-start and race-start authorize autonomous motion only while this command keeps
running. It renews authorization every 0.2 s; the supervisor stops the car if renewals
stop. With --heartbeat-stdin, renewals continue only while the laptop keeps writing lines
to standard input, so a lost laptop or network ends authorization even if the SSH
session lingers on the Jetson. Ctrl+C sends stop immediately.
"""

import argparse
import json
import queue
import sys
import threading
import time

from .protocol import Client


def print_status(status, as_json=False):
    if as_json:
        print(json.dumps(status, indent=2))
        return
    print(f"State:   {status.get('state')}   session {status.get('session')}")
    print(f"Map:     {'validated' if status.get('map_ready') else 'not ready'}")
    for reason in status.get("reasons") or []:
        print(f"Reason:  {reason}")
    readiness = status.get("race_readiness") or []
    print("Race:    READY to start" if not readiness and status.get("map_ready") else
          "Race:    not ready" + "".join(f"\n  - {r}" for r in readiness))
    localization = status.get("localization") or {}
    if localization:
        print("Localization: " + ", ".join(
            f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}"
            for k, v in localization.items() if k != "failures"))


def stdin_lines():
    """Queue each laptop heartbeat line; None marks end of input."""
    lines = queue.Queue()

    def reader():
        for line in sys.stdin:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=reader, daemon=True).start()
    return lines


def authorize(client, session, laptop=None, period_s=0.2, laptop_timeout_s=0.5,
              clock=time.monotonic, sleep=time.sleep):
    """Renew authorization until the session ends, the laptop goes silent or Ctrl+C.

    laptop is a queue of heartbeat lines (None = closed), or None to rely on this
    terminal alone. Return an exit code: 0 when the supervisor ended the session itself
    (for example after a stop), 3 when the laptop link was lost, 130 on Ctrl+C.
    """
    last_laptop = clock()
    try:
        while True:
            if laptop is not None:
                try:
                    while True:
                        item = laptop.get_nowait()
                        if item is None:
                            client.request("stop", reason="laptop connection closed")
                            print("Laptop connection closed: stop sent", file=sys.stderr)
                            return 3
                        last_laptop = clock()
                except queue.Empty:
                    pass
                if clock() - last_laptop > laptop_timeout_s:
                    client.request("stop", reason="laptop heartbeats stopped")
                    print("Laptop heartbeats stopped: stop sent", file=sys.stderr)
                    return 3
            reply = client.request("heartbeat", session=session)
            if not reply.get("ok"):
                print(f"Authorization ended: state {reply.get('state')}", file=sys.stderr)
                return 0
            sleep(period_s)
    except KeyboardInterrupt:
        client.request("stop", reason="operator interrupted the authorizing command")
        print("Stopped", file=sys.stderr)
        return 130


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("status", "map-start", "race-start", "stop"))
    parser.add_argument("--session", help="Session shown by status; defaults to the current one")
    parser.add_argument("--heartbeat-stdin", action="store_true",
                        help="Authorize only while lines keep arriving on stdin (SSH use)")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--socket", help="Supervisor socket path")
    args = parser.parse_args(argv)
    try:
        client = Client(args.socket)
    except ConnectionError as exc:
        print(exc, file=sys.stderr)
        return 2
    try:
        if args.command == "status":
            print_status(client.request("status"), args.json)
            return 0
        if args.command == "stop":
            reply = client.request("stop", reason="stop command")
            print(f"{reply['message']} ({reply['state']})")
            return 0
        laptop = stdin_lines() if args.heartbeat_stdin else None
        session = args.session or client.request("status")["session"]
        reply = client.request(args.command, session=session)
        if not reply["ok"]:
            print(f"Rejected: {reply['message']}", file=sys.stderr)
            for reason in reply.get("reasons", []):
                print(f"  - {reason}", file=sys.stderr)
            return 1
        print(f"{reply['message']}; session {reply['session']}. Ctrl+C to stop.", flush=True)
        return authorize(client, reply["session"], laptop)
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
