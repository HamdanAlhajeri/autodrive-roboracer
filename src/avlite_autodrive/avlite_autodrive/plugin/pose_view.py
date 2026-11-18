"""AVLite task: show the estimated pose next to the simulator's true pose in a browser.

A ``TaskStrategy`` runs inside the executer loop, like AVLite's own tasks. Each cycle it reads
the localizer's pose (``executer.ego_state``) and the bridge's ground-truth diagnostic, and
publishes both to a small web page. It only reads; it never touches the car's commands, and
any failure disables the view instead of the controller.
"""

import logging
import time

from avlite.c40_execution.c43_task_strategy import TaskSchedule, TaskStrategy

from avlite_autodrive.pose_view import PoseHub, PoseViewServer, build_snapshot

log = logging.getLogger(__name__)


class PoseComparisonView(TaskStrategy):
    """Publish estimate-versus-truth snapshots every control cycle."""

    schedule = TaskSchedule.EVERY_CYCLE

    def __init__(self, config, geometry=None, localizer=None):
        """``localizer`` (optional) supplies the scan-alignment score and hit count."""
        self.hub = PoseHub(geometry, config.error_alert_m)
        self.server = PoseViewServer(self.hub, config)
        self.localizer = localizer
        self._started = False
        self._failed = False

    def start(self):
        """Bind the web server now, so the page loads before the car is ready. Idempotent.

        Raises OSError when the port is taken; the caller decides whether that is fatal.
        """
        if not self._started:
            self.server.start()
            self._started = True

    def execute(self, executer, event=None):
        if self._failed:
            return
        try:
            self.start()
            ego = executer.ego_state
            truth = getattr(executer.world, "true_pose", lambda: None)()
            snapshot = build_snapshot(
                time.monotonic(), (ego.x, ego.y, ego.theta), truth,
                alignment=getattr(self.localizer, "alignment", None),
                hits=getattr(self.localizer, "hits", None),
                speed=getattr(ego, "velocity", None))
            if snapshot is not None:
                self.hub.publish(snapshot)
        except Exception as error:  # the view must never stop the car
            self._failed = True
            log.warning("Pose view disabled: %s", error)

    def reset(self):
        """The simulator was reset: the car is back at the start, so drop the old trails."""
        self.hub.reset()

    def close(self):
        self.server.close()
