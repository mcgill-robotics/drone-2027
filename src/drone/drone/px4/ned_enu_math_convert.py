"""
The maths for switching between PX4's axes (NED) and ours (ENU).

PX4 and ROS point their x, y, z axes in different directions:

    PX4 (NED):  x = North, y = East, z = Down.  Yaw 0 = facing North, turning clockwise.
    Ours (ENU): x = East,  y = North, z = Up.   Yaw 0 = facing East, turning counter-clockwise.

So "3 m up" is z = -3 for PX4 but z = 3 for us. Our code uses ENU everywhere, as it
did with MAVROS, which used to do this conversion for us. PX4 values are converted
here on the way in and out.

The drone's own axes differ the same way: PX4 uses forward-right-down (FRD), ROS
uses forward-left-up (FLU).

No ROS imports, so the unit tests can run it anywhere.
"""

import math


def wrap_pi(angle):
    """Bring an angle in radians into the range -pi..pi (e.g. 3*pi/2 -> -pi/2)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def ned_to_enu(x, y, z):
    """PX4 (north, east, down) -> ours (east, north, up). For positions and velocities."""
    return y, x, -z


def enu_to_ned(x, y, z):
    """Ours (east, north, up) -> PX4 (north, east, down). For positions and velocities."""
    return y, x, -z


def yaw_ned_to_enu(yaw):
    """PX4 heading (0 = North, clockwise) -> our yaw (0 = East, counter-clockwise)."""
    return wrap_pi(math.pi / 2.0 - yaw)


def yaw_enu_to_ned(yaw):
    """Our yaw (0 = East, counter-clockwise) -> PX4 heading (0 = North, clockwise)."""
    return wrap_pi(math.pi / 2.0 - yaw)


def yaw_rate_ned_to_enu(rate):
    """PX4 turn rate -> ours. Same spin, opposite sign, because PX4's z axis points down."""
    return -rate


def yaw_rate_enu_to_ned(rate):
    """Our turn rate -> PX4's. Same spin, opposite sign, because PX4's z axis points down."""
    return -rate


def quat_wxyz_to_euler(w, x, y, z):
    """
    Quaternion (w, x, y, z) -> (roll, pitch, yaw) in radians.

    A quaternion is the 4-number way PX4 stores which way the drone is tilted and
    facing; roll/pitch/yaw are the same thing as three angles. The result is in
    the same axes as the input: for PX4's attitude, that is NED.
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
    PX4 (roll, pitch, yaw) -> ours.

    Roll stays the same: right wing down is positive in both.
    Pitch flips sign: nose up is positive for PX4, negative for ROS.
    Yaw is converted with yaw_ned_to_enu().
    """
    return roll, -pitch, yaw_ned_to_enu(yaw)
