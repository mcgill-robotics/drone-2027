"""
Turns PX4's messages into simple Python dicts, converted to ENU.

telemetry.py's getters use these. Each function takes one PX4 message and returns
a dict like {"x": ..., "y": ..., "z": ...}, or None if PX4 says the data isn't
valid right now (e.g. no GPS fix yet). The dict keys match drone-2026's getters.

No ROS imports, so the unit tests can run it anywhere (they pass in fake messages).
"""

import math

from drone.px4.ned_enu_math_convert import (
    ned_to_enu,
    quat_wxyz_to_euler,
    yaw_ned_to_enu,
)


def _finite(*values):
    """True if every value is a real number (not None, NaN or infinity)."""
    return all(v is not None and math.isfinite(float(v)) for v in values)


def local_position_enu(msg):
    """PX4 position -> {"x": east, "y": north, "z": up} in metres, or None if invalid."""
    if not (msg.xy_valid and msg.z_valid) or not _finite(msg.x, msg.y, msg.z):
        return None
    x, y, z = ned_to_enu(float(msg.x), float(msg.y), float(msg.z))
    return {"x": x, "y": y, "z": z}


def local_velocity_enu(msg):
    """PX4 velocity -> {"x": east, "y": north, "z": up} in m/s, or None if invalid."""
    if not (msg.v_xy_valid and msg.v_z_valid) or not _finite(msg.vx, msg.vy, msg.vz):
        return None
    x, y, z = ned_to_enu(float(msg.vx), float(msg.vy), float(msg.vz))
    return {"x": x, "y": y, "z": z}


def heading_enu(msg):
    """PX4 heading -> our yaw in radians (0 = East, counter-clockwise), or None."""
    if not _finite(msg.heading):
        return None
    return yaw_ned_to_enu(float(msg.heading))


def attitude_ned(msg):
    """PX4 attitude -> (roll, pitch, yaw) in radians, still in PX4's NED convention."""
    w, x, y, z = (float(v) for v in msg.q)
    if not _finite(w, x, y, z) or (w == 0.0 and x == 0.0 and y == 0.0 and z == 0.0):
        return None
    return quat_wxyz_to_euler(w, x, y, z)


def global_position(msg):
    """PX4 GPS position -> {"latitude", "longitude", "altitude" (m above sea level)}."""
    if not msg.lat_lon_valid:
        return None
    return {
        "latitude": float(msg.lat),
        "longitude": float(msg.lon),
        "altitude": float(msg.alt) if msg.alt_valid else None,
    }


def home_position(msg):
    """PX4 home position -> {"latitude", "longitude", "altitude" (m above sea level)}."""
    if not msg.valid_hpos:
        return None
    return {
        "latitude": float(msg.lat),
        "longitude": float(msg.lon),
        "altitude": float(msg.alt) if msg.valid_alt else None,
    }


def battery(msg):
    """
    PX4 battery -> {"voltage", "current", "percentage", "remaining"}.

    `percentage` and `remaining` are the same value: charge left as 0..1 (not
    0..100), or None if unknown. Both keys exist because MAVROS used them.
    """
    remaining = float(msg.remaining) if msg.remaining >= 0.0 else None
    return {
        "voltage": float(msg.voltage_v),
        "current": float(msg.current_a),
        "percentage": remaining,
        "remaining": remaining,
    }


def gps_raw(msg):
    """PX4 raw GPS -> fix type, satellites, accuracy etc., as drone-2026's get_gps_raw() gave."""
    return {
        "fix_type": int(msg.fix_type),
        "lat": float(msg.latitude_deg),
        "lon": float(msg.longitude_deg),
        "alt": float(msg.altitude_msl_m),
        "eph": float(msg.eph),
        "epv": float(msg.epv),
        "vel": float(msg.vel_m_s),
        "cog": float(msg.cog_rad),
        "satellites_visible": int(msg.satellites_used),
    }
