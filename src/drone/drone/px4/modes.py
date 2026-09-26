"""
PX4 doesn't understand words like "OFFBOARD" or "LAND".
It only understands numbers. modes.py is a translator between names and PX4's numbers.

  - Changing mode: mode_command("LAND") gives the command numbers to send to PX4.
  - Reading mode:  nav_state_names() lets get_mode() turn PX4's mode number back
                   into a name like "OFFBOARD".
  - Nicknames:     normalize_mode_name() makes "land", "LAND" and "auto_land" all
                   mean the same thing, and "HOLD" mean PX4's "AUTO.LOITER".

Names match the ones MAVROS used, so `px4.get_mode() == "OFFBOARD"` still works.
Also holds the command numbers for arm, land, etc. that commands.py sends.

No ROS imports, so the unit tests can check every translation on any laptop.
"""

import math

# PX4's command numbers (checked against px4_msgs release/1.17 VehicleCommand.msg).
VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_DO_SET_ACTUATOR = 187
VEHICLE_CMD_DO_MOUNT_CONTROL = 205
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

# First parameter of a set-mode command: 1 means "the next two numbers are a PX4 mode".
CUSTOM_MODE_ENABLED = 1.0

# Each mode name -> PX4's (main mode, sub mode) numbers. AUTO modes share main mode 4.
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

# Nicknames -> PX4's real mode name.
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
    """Tidy a mode name: "auto_rtl", "AUTO.RTL" and "rtl" all become "AUTO.RTL"."""
    key = str(name).strip().upper().replace("_", ".")
    return _ALIASES.get(key, key)


def mode_command(name):
    """
    The command to send to PX4 to switch to mode `name`, as (command number, 7 params).

    RTL and LAND have their own commands (NaN in the params means "from where you
    are now"); every other mode uses the general set-mode command (176).
    Raises ValueError for a name PX4 doesn't have.
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
    Build the table from PX4's mode numbers to names, e.g. {14: "OFFBOARD", ...}.

    Reads the numbers from the VehicleStatus message type instead of typing them
    in, so the table stays right if a newer px4_msgs renumbers the modes.
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
