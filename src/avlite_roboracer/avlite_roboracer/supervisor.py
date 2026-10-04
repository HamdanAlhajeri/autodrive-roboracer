"""Jetson supervisor state machine (no ROS, no threads; time is passed in).

    IDLE -> MAPPING (map-start) -> FINALIZING (lap complete, car stopped)
    FINALIZING -> READY (map, plan and localization validated) | STOPPED (validation failed)
    READY -> RACING (race-start) ; any active state -> STOPPED (stop, fault, lost laptop)

Rules enforced here:
- a race-start that fails readiness is rejected with the reasons and never queued;
- completing the mapping lap never starts racing;
- every motion-starting command and heartbeat must name the current session, so a delayed
  command cannot act on a later session; sessions include a per-boot id, so commands from
  before a reboot or supervisor restart are always rejected;
- restarts begin in IDLE and never resume movement;
- stop is accepted in every state and may be repeated.

The runtime executes the returned actions (start/stop controllers and SLAM processes).
"""

from collections import deque
from enum import Enum


class State(str, Enum):
    IDLE = "IDLE"
    MAPPING = "MAPPING"
    FINALIZING = "FINALIZING"
    READY = "READY"
    RACING = "RACING"
    STOPPED = "STOPPED"


MOTION_STATES = (State.MAPPING, State.RACING)


class Action(str, Enum):
    START_MAPPING = "start_mapping"
    STOP_AFTER_LAP = "stop_after_lap"
    FINALIZE = "finalize"
    START_RACING = "start_racing"
    STOP_MOTION = "stop_motion"
    ABORT_FINALIZE = "abort_finalize"


class Supervisor:
    def __init__(self, boot_id, authorization_timeout_s, readiness, mapping_checks,
                 previous=None):
        """readiness(now) and mapping_checks(now) return lists of failure reasons.

        previous is an earlier to_dict() snapshot; it is reported in status but never used
        to resume a session.
        """
        if authorization_timeout_s <= 0:
            raise ValueError("authorization timeout must be positive")
        self.boot_id = str(boot_id)
        self.authorization_timeout_s = authorization_timeout_s
        self.readiness = readiness
        self.mapping_checks = mapping_checks
        self.state = State.IDLE
        self.number = 0
        self.reasons = []
        self.plan = None
        self.lap_complete = False
        self.last_heartbeat_s = None
        self.actions = deque()
        self.history = deque(maxlen=50)
        self.previous = None
        if previous:
            self.previous = {"state": previous.get("state"), "session": previous.get("session"),
                             "reasons": previous.get("reasons", [])}
            self.reasons = [f"supervisor restarted (previously {previous.get('state')}); "
                            "no session was resumed"]

    # ----------------------------------------------------------------- helpers
    @property
    def session(self):
        return f"{self.boot_id}-{self.number}"

    def _transition(self, state, now, reasons=None, actions=()):
        self.history.append({"t": now, "from": self.state.value, "to": state.value,
                             "reasons": list(reasons or [])})
        self.state = state
        if reasons is not None:
            self.reasons = list(reasons)
        self.actions.extend(actions)

    def _reply(self, ok, message, reasons=None):
        return {"ok": ok, "message": message, "state": self.state.value,
                "session": self.session, "reasons": list(reasons or [])}

    def take_actions(self):
        """Return and clear pending runtime actions, oldest first."""
        actions = list(self.actions)
        self.actions.clear()
        return actions

    # ---------------------------------------------------------------- commands
    def map_start(self, session, now):
        """Begin a new mapping session from IDLE or STOPPED, if checks pass."""
        if session != self.session:
            return self._reply(False, "stale session: read status and retry", ["stale session"])
        if self.state not in (State.IDLE, State.STOPPED):
            return self._reply(False, f"map-start not allowed in {self.state.value}")
        failures = self.mapping_checks(now)
        if failures:
            return self._reply(False, "mapping preconditions failed", failures)
        self.number += 1
        self.plan = None
        self.lap_complete = False
        self.last_heartbeat_s = now
        self._transition(State.MAPPING, now, [], [Action.START_MAPPING])
        return self._reply(True, "mapping started")

    def race_start(self, session, now):
        """Start racing on the current validated map; rejected (never queued) if not ready."""
        if session != self.session:
            return self._reply(False, "stale session: read status and retry", ["stale session"])
        if self.state == State.RACING:
            return self._reply(False, "already racing; duplicate race-start ignored")
        if self.state not in (State.READY, State.STOPPED):
            return self._reply(False, f"race-start not allowed in {self.state.value}",
                               [f"state is {self.state.value}"])
        if self.plan is None:
            return self._reply(False, "no validated map; run map-start",
                               ["no validated map and plan"])
        failures = self.readiness(now)
        if failures:
            return self._reply(False, "not ready", failures)
        self.last_heartbeat_s = now
        self._transition(State.RACING, now, [], [Action.START_RACING])
        return self._reply(True, "racing started")

    def heartbeat(self, session, now):
        """Renew laptop authorization for the current motion session."""
        if session != self.session or self.state not in MOTION_STATES:
            return self._reply(False, "authorization not renewed",
                               ["session ended or changed"])
        self.last_heartbeat_s = now
        return self._reply(True, "authorized")

    def stop(self, now, reason="stop command"):
        """Stop motion from any state; repeated calls succeed without side effects."""
        if self.state in (State.IDLE, State.STOPPED):
            self.actions.append(Action.STOP_MOTION)
            return self._reply(True, "already stopped")
        self.actions.clear()  # a stop supersedes starts not yet executed by the runtime
        self.number += 1  # revoke commands and heartbeats issued before this stop
        self.last_heartbeat_s = None
        actions = [Action.STOP_MOTION]
        if self.state == State.FINALIZING:
            actions.append(Action.ABORT_FINALIZE)
            self.plan = None
        self._transition(State.STOPPED, now, [reason], actions)
        return self._reply(True, "stopped")

    # ------------------------------------------------------------------ events
    def lap_completed(self, now):
        """Mapping lap detected: decelerate to a stop; racing is never started here."""
        if self.state == State.MAPPING and not self.lap_complete:
            self.lap_complete = True
            self.actions.append(Action.STOP_AFTER_LAP)
            self.history.append({"t": now, "event": "lap_completed"})

    def vehicle_stopped(self, now):
        """Car is stationary after the completed lap: finalize while stopped."""
        if self.state == State.MAPPING and self.lap_complete:
            self._transition(State.FINALIZING, now, [], [Action.FINALIZE])

    def finalize_succeeded(self, plan, now):
        if self.state == State.FINALIZING:
            self.plan = plan
            self._transition(State.READY, now, [])

    def finalize_failed(self, reasons, now):
        """Validation failed: stay stopped and require a new mapping attempt."""
        if self.state == State.FINALIZING:
            self.plan = None
            self.actions.clear()
            self.number += 1
            self.last_heartbeat_s = None
            self._transition(State.STOPPED, now,
                             list(reasons) + ["map rejected: run map-start for another attempt"],
                             [Action.STOP_MOTION])

    def fault(self, reason, now):
        """Stop on a fault in any active state."""
        if self.state in (State.IDLE, State.STOPPED):
            return
        self.stop(now, reason)

    def tick(self, now):
        """Expire laptop authorization during autonomous motion."""
        if self.state in MOTION_STATES and (
                self.last_heartbeat_s is None
                or not 0 <= now - self.last_heartbeat_s <= self.authorization_timeout_s):
            self.fault("laptop authorization expired", now)

    # ------------------------------------------------------------------ status
    def status(self, now, extra=None):
        readiness = self.readiness(now) if self.plan is not None else ["no validated map"]
        age = None if self.last_heartbeat_s is None else now - self.last_heartbeat_s
        return {"state": self.state.value, "session": self.session, "reasons": self.reasons,
                "map_ready": self.plan is not None, "plan": self.plan,
                "race_readiness": readiness,
                "authorization_age_s": age if self.state in MOTION_STATES else None,
                "previous_run": self.previous, **(extra or {})}

    def to_dict(self):
        return {"state": self.state.value, "session": self.session, "reasons": self.reasons,
                "history": list(self.history)}
