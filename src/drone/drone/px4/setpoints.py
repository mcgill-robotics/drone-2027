"""
Field values for PX4 offboard messages, built from ENU inputs.

OffboardControlMode says which kind of control we are doing and doubles as the
"companion computer is alive" heartbeat. TrajectorySetpoint carries the numbers
in NED; NaN means "do not control this value".

Pure Python: returns plain dicts that offboard.py copies onto the ROS messages.
"""

import math

from drone.px4.frames import enu_to_ned, yaw_enu_to_ned, yaw_rate_enu_to_ned

NAN = math.nan
CONTROL_KINDS = ("position", "velocity")


def _nan3():
    return [NAN, NAN, NAN]


def position_setpoint(x, y, z, yaw=None):
    """ENU position in metres and ENU yaw in radians (None = leave yaw alone) -> TrajectorySetpoint fields."""
    return {
        "position": list(enu_to_ned(float(x), float(y), float(z))),
        "velocity": _nan3(),
        "acceleration": _nan3(),
        "yaw": NAN if yaw is None else yaw_enu_to_ned(float(yaw)),
        "yawspeed": NAN,
    }


def velocity_setpoint(vx, vy, vz, yaw_rate=0.0):
    """ENU velocity in m/s and ENU yaw rate in rad/s -> TrajectorySetpoint fields."""
    return {
        "position": _nan3(),
        "velocity": list(enu_to_ned(float(vx), float(vy), float(vz))),
        "acceleration": _nan3(),
        "yaw": NAN,
        "yawspeed": yaw_rate_enu_to_ned(float(yaw_rate)),
    }


def control_mode_flags(kind):
    """OffboardControlMode fields for a "position" or "velocity" setpoint."""
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
