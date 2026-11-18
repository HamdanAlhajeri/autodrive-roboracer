"""AVLite plugin: the simulator bridge, the controllers and the pose view task.

Members load on first access, so the Jetson hardware process can import the controllers
without importing the simulator bridge or its ROS topics. Importing a member registers it
with AVLite as before.
"""

from importlib import import_module

_MEMBERS = {"AutoDRIVEWorldBridge": ".bridge", "AutoDRIVEFollowTheGap": ".controller",
            "PoseComparisonView": ".pose_view"}

__all__ = list(_MEMBERS)


def __getattr__(name):
    if name not in _MEMBERS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    return getattr(import_module(_MEMBERS[name], __name__), name)
