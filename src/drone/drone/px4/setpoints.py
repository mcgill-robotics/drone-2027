"""
Fills in the numbers for "fly here" and "fly this fast" orders (see offboard.py).

You give ENU values (x East, y North, z Up); these functions convert them to NED,
which is what PX4 expects. Any field set to NaN means "PX4, don't control this":
a position order leaves velocity as NaN, and a velocity order leaves position as NaN.

Returns plain dicts; offboard.py copies them onto the real ROS messages. No ROS
imports, so the unit tests can run it anywhere.
"""

import math

from drone.px4.ned_enu_math_convert import (
    enu_to_ned,
    yaw_enu_to_ned,
    yaw_rate_enu_to_ned,
)

NAN = math.nan
CONTROL_KINDS = ("position", "velocity")  # the two kinds of order we support


def _nan3():
    """[NaN, NaN, NaN]: "don't control" for all three axes."""
    return [NAN, NAN, NAN]


def position_setpoint(x, y, z, yaw=None):
    """
    Fields for "fly to (x, y, z) facing `yaw`".

    x, y, z in metres (ENU); yaw in radians (0 = East), or None to not control it.
    """
    return {
        "position": list(enu_to_ned(float(x), float(y), float(z))),
        "velocity": _nan3(),
        "acceleration": _nan3(),
        "yaw": NAN if yaw is None else yaw_enu_to_ned(float(yaw)),
        "yawspeed": NAN,
    }


def velocity_setpoint(vx, vy, vz, yaw_rate=0.0):
    """
    Fields for "fly at (vx, vy, vz), turning at `yaw_rate`".

    vx, vy, vz in m/s (ENU); yaw_rate in rad/s, positive = counter-clockwise.
    """
    return {
        "position": _nan3(),
        "velocity": list(enu_to_ned(float(vx), float(vy), float(vz))),
        "acceleration": _nan3(),
        "yaw": NAN,
        "yawspeed": yaw_rate_enu_to_ned(float(yaw_rate)),
    }


def control_mode_flags(kind):
    """Fields for OffboardControlMode: tells PX4 whether this is a position or velocity order."""
    if kind not in CONTROL_KINDS:
        raise ValueError(f"Unsupported offboard control kind {kind!r}")
    return {
        "position": kind == "position",
        "velocity": kind == "velocity",
        "acceleration": False,
        "attitude": False,
        "body_rate": False,
        "thrust_and_torque": False,
        "direct_actuator": False,
    }
