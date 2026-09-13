"""
PX4 flight modes: VehicleStatus.nav_state <-> names, and the command that selects each mode.

Mode names follow the strings MAVROS used ("OFFBOARD", "POSCTL", "AUTO.RTL", ...)
so comparisons like `px4.get_mode() == "OFFBOARD"` read the same as before.
Short aliases such as "RTL", "HOLD" and "LAND" are accepted when changing mode.

Pure Python: no ROS imports, so it can be unit tested anywhere.
"""

import math

# VehicleCommand ids, checked against PX4 release/1.16 msg/versioned/VehicleCommand.msg.
VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_DO_SET_ACTUATOR = 187
VEHICLE_CMD_DO_MOUNT_CONTROL = 205
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

# DO_SET_MODE param1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, i.e. "param2/param3 are PX4 modes".
CUSTOM_MODE_ENABLED = 1.0

# PX4 custom (main mode, sub mode) pairs for DO_SET_MODE param2/param3.
_SET_MODE_TARGETS = {
    "MANUAL": (1.0, 0.0),
    "ALTCTL": (2.0, 0.0),
    "POSCTL": (3.0, 0.0),
    "ACRO": (5.0, 0.0),
    "OFFBOARD": (6.0, 0.0),
    "STABILIZED": (7.0, 0.0),
    "AUTO.TAKEOFF": (4.0, 2.0),
    "AUTO.LOITER": (4.0, 3.0),
    "AUTO.MISSION": (4.0, 4.0),
}

_ALIASES = {
    "HOLD": "AUTO.LOITER",
    "LOITER": "AUTO.LOITER",
    "MISSION": "AUTO.MISSION",
    "TAKEOFF": "AUTO.TAKEOFF",
    "RTL": "AUTO.RTL",
    "LAND": "AUTO.LAND",
    "POSITION": "POSCTL",
    "ALTITUDE": "ALTCTL",
    "STAB": "STABILIZED",
}


def normalize_mode_name(name):
    """ "auto_rtl" / "AUTO.RTL" / "rtl" -> "AUTO.RTL"."""
    key = str(name).strip().upper().replace("_", ".")
    return _ALIASES.get(key, key)


def mode_command(name):
    """
    Return (command_id, params) that asks PX4 to enter `name`.

    params is a tuple for param1..param7. RTL and LAND use their dedicated NAV
    commands with NaN position/yaw ("use the current one"); every other mode
    goes through DO_SET_MODE. Raises ValueError for unknown names.
    """
    key = normalize_mode_name(name)
    nan = math.nan
    if key == "AUTO.RTL":
        return VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    if key == "AUTO.LAND":
        return VEHICLE_CMD_NAV_LAND, (0.0, 0.0, 0.0, nan, nan, nan, nan)
    if key in _SET_MODE_TARGETS:
        main_mode, sub_mode = _SET_MODE_TARGETS[key]
        return VEHICLE_CMD_DO_SET_MODE, (
            CUSTOM_MODE_ENABLED,
            main_mode,
            sub_mode,
            0.0,
            0.0,
            0.0,
            0.0,
        )
    raise ValueError(f"Unknown PX4 mode {name!r}")


def nav_state_names(status_type):
    """
    Build {nav_state value: mode name} from a VehicleStatus class's NAVIGATION_STATE_* constants.

    Reading the constants from the message class (instead of copying numbers)
    keeps this right if a newer px4_msgs adds or renumbers states.
    """
    attrs = set(dir(status_type)) | set(dir(type(status_type)))
    names = {}
    for attr in attrs:
        if not attr.startswith("NAVIGATION_STATE_") or attr == "NAVIGATION_STATE_MAX":
            continue
        name = attr[len("NAVIGATION_STATE_") :]
        if name.startswith("AUTO_"):
            name = "AUTO." + name[len("AUTO_") :]
        elif name == "STAB":
            name = "STABILIZED"
        names[int(getattr(status_type, attr))] = name
    return names
