"""
Turn PX4 telemetry messages into the ENU dicts the rest of the code expects.

Each function takes a px4_msgs message (or anything with the same attribute
names) and returns a plain dict, or None when PX4 marks the data invalid. The key
names match drone-2026's getters so callers did not have to change.

Pure Python: no ROS imports.
"""

import math

from drone.px4.frames import ned_to_enu, quat_wxyz_to_euler, yaw_ned_to_enu


def _finite(*values):
    return all(v is not None and math.isfinite(float(v)) for v in values)


def local_position_enu(msg):
    """VehicleLocalPosition -> {"x": east, "y": north, "z": up} in metres."""
    if not (msg.xy_valid and msg.z_valid) or not _finite(msg.x, msg.y, msg.z):
        return None
    x, y, z = ned_to_enu(float(msg.x), float(msg.y), float(msg.z))
    return {"x": x, "y": y, "z": z}


def local_velocity_enu(msg):
    """VehicleLocalPosition -> {"x": east, "y": north, "z": up} in m/s."""
    if not (msg.v_xy_valid and msg.v_z_valid) or not _finite(msg.vx, msg.vy, msg.vz):
        return None
    x, y, z = ned_to_enu(float(msg.vx), float(msg.vy), float(msg.vz))
    return {"x": x, "y": y, "z": z}


def heading_enu(msg):
    """VehicleLocalPosition.heading -> ENU yaw in radians (0 = facing East)."""
    if not _finite(msg.heading):
        return None
    return yaw_ned_to_enu(float(msg.heading))


def attitude_ned(msg):
    """VehicleAttitude.q (w, x, y, z; FRD body -> NED) -> (roll, pitch, yaw) in radians."""
    w, x, y, z = (float(v) for v in msg.q)
    if not _finite(w, x, y, z) or (w == 0.0 and x == 0.0 and y == 0.0 and z == 0.0):
        return None
    return quat_wxyz_to_euler(w, x, y, z)


def global_position(msg):
    """VehicleGlobalPosition -> {"latitude", "longitude", "altitude" (AMSL metres)}."""
    if not msg.lat_lon_valid:
        return None
    return {
        "latitude": float(msg.lat),
        "longitude": float(msg.lon),
        "altitude": float(msg.alt) if msg.alt_valid else None,
    }


def home_position(msg):
    """HomePosition -> {"latitude", "longitude", "altitude" (AMSL metres)}."""
    if not msg.valid_hpos:
        return None
    return {
        "latitude": float(msg.lat),
        "longitude": float(msg.lon),
        "altitude": float(msg.alt) if msg.valid_alt else None,
    }


def battery(msg):
    """
    BatteryStatus -> {"voltage", "current", "percentage", "remaining"}.

    `percentage` and `remaining` are both the 0..1 fraction PX4 reports (None if
    unknown), matching sensor_msgs/BatteryState.percentage that MAVROS gave us.
    """
    remaining = float(msg.remaining) if msg.remaining >= 0.0 else None
    return {
        "voltage": float(msg.voltage_v),
        "current": float(msg.current_a),
        "percentage": remaining,
        "remaining": remaining,
    }


def gps_raw(msg):
    """SensorGps -> the raw GPS dict drone-2026's get_gps_raw() returned."""
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
