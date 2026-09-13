"""
Coordinate-frame conversions between PX4 (NED / FRD) and ROS (ENU / FLU).

PX4 reports and accepts everything in NED: x = North, y = East, z = Down, with
body axes FRD (forward, right, down). Yaw 0 faces North and grows clockwise.

The rest of this codebase works in ENU: x = East, y = North, z = Up, with body
axes FLU (forward, left, up). Yaw 0 faces East and grows counter-clockwise.
MAVROS used to convert to ENU for us, so the mission code was written against it.

This module is the only place that knows both conventions. It imports nothing
from ROS so it can be unit tested anywhere.
"""

import math


def wrap_pi(angle):
    """Wrap an angle in radians into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def ned_to_enu(x, y, z):
    """(north, east, down) -> (east, north, up). Works for positions and velocities."""
    return y, x, -z


def enu_to_ned(x, y, z):
    """(east, north, up) -> (north, east, down). Works for positions and velocities."""
    return y, x, -z


def yaw_ned_to_enu(yaw):
    """PX4 heading (0 = North, clockwise) -> ENU yaw (0 = East, counter-clockwise)."""
    return wrap_pi(math.pi / 2.0 - yaw)


def yaw_enu_to_ned(yaw):
    """ENU yaw (0 = East, counter-clockwise) -> PX4 heading (0 = North, clockwise)."""
    return wrap_pi(math.pi / 2.0 - yaw)


def yaw_rate_ned_to_enu(rate):
    """Yaw rate about Down -> yaw rate about Up: same spin, opposite sign."""
    return -rate


def yaw_rate_enu_to_ned(rate):
    """Yaw rate about Up -> yaw rate about Down: same spin, opposite sign."""
    return -rate


def quat_wxyz_to_euler(w, x, y, z):
    """
    Quaternion (w, x, y, z) -> (roll, pitch, yaw) in radians, ZYX convention.

    The angles are expressed in whatever frames the quaternion relates. For
    PX4's VehicleAttitude.q that is the FRD body relative to NED.
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def euler_ned_to_enu(roll, pitch, yaw):
    """
    FRD/NED (roll, pitch, yaw) -> FLU/ENU (roll, pitch, yaw).

    Roll keeps its sign (right wing down is positive in both). Pitch flips:
    nose-up is positive in FRD and negative in FLU. Yaw goes through
    yaw_ned_to_enu.
    """
    return roll, -pitch, yaw_ned_to_enu(yaw)
