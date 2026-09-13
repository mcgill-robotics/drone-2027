#!/usr/bin/env python3
"""
Mission 1 PX4 movement helpers: connect, take off, fly to GPS coordinates, land.

Moved from drone-2026 tests/mission1/movement.py. The flight logic is unchanged
because drone.px4 still speaks ENU. What changed with the move from MAVROS to
the uXRCE-DDS bridge:
  - setup_environment() starts MicroXRCEAgent instead of launching MAVROS
  - PX4 speed limits are no longer set from code; set them in QGC (docs/px4_setup.md)
  - nothing here spins ROS; the PX4 interface does that on its own thread
  - the setpoint heartbeat starts before OFFBOARD is requested, not after
"""

from __future__ import annotations

import math
import time
from typing import Any, Callable

EARTH_RADIUS_M = 6_378_137.0
ARRIVAL_TOLERANCE_M = 0.5
ALTITUDE_TOLERANCE_M = 0.75
SETPOINT_RATE_HZ = 10

# Must be set in QGroundControl before flying Mission 1 (drone-2026 set these from code).
REQUIRED_MOTION_LIMITS = (
    ("MPC_XY_VEL_MAX", 2.0),
    ("MPC_XY_CRUISE", 1.0),
    ("MPC_VEL_MANUAL", 2.0),
    ("MPC_LAND_SPEED", 0.4),
    ("MPC_LAND_CRWL", 0.2),
)

_px4 = None
_launch_alt = None
_hover_alt = None


def gps_to_local_offset(
    origin_lat: float, origin_lon: float, target_lat: float, target_lon: float
) -> tuple[float, float]:
    origin_lat_rad = math.radians(origin_lat)
    north = math.radians(target_lat - origin_lat) * EARTH_RADIUS_M
    east = (
        math.radians(target_lon - origin_lon)
        * EARTH_RADIUS_M
        * math.cos(origin_lat_rad)
    )
    return east, north


def setup_environment(link_args: Any, *, boot: bool = True) -> bool:
    """Start MicroXRCEAgent once for the whole mission. Phase scripts then connect with --no-agent."""
    if not boot:
        return True

    from drone.px4.agent import start_agent_from_args

    print("[MISSION1 SETUP] Starting MicroXRCEAgent")
    return start_agent_from_args(link_args)


def connect():
    global _px4

    if _px4 is not None:
        return _px4

    from drone.px4.interface import init_px4

    print("[MISSION1 MOVE] Initializing PX4Interface")
    _px4 = init_px4()
    if not _px4.connected:
        raise RuntimeError("Failed to connect to PX4 through MicroXRCEAgent")
    return _px4


def current_gps_coordinate(timeout: float = 10.0) -> dict[str, float] | None:
    """Return the current GPS lat/lon, waiting up to `timeout` for a valid fix."""
    px4 = connect()

    start_time = time.time()
    while (time.time() - start_time) < timeout:
        gps = px4.get_gps_location()
        if gps is not None:
            return {"lat": float(gps["latitude"]), "lon": float(gps["longitude"])}
        time.sleep(0.1)

    return None


def print_motion_limit_reminder() -> None:
    print("[MISSION1 MOVE] These PX4 motion limits must already be set in QGC:")
    for name, value in REQUIRED_MOTION_LIMITS:
        print(f"    {name} = {value}")


def begin_phase(
    *, takeoff_altitude: float = 5.0, arm_timeout: float = 60.0, api_arm: bool = False
) -> bool:
    """Prepare OFFBOARD movement and take off for one mission phase."""
    global _launch_alt, _hover_alt

    px4 = connect()
    print_motion_limit_reminder()

    print("[MISSION1 MOVE] Starting background setpoint stream")
    px4.start_offboard_stream_background()

    print("[MISSION1 MOVE] Switching to OFFBOARD")
    if not px4.start_offboard():
        return False

    if api_arm:
        print("[MISSION1 MOVE] Arming from code (simulator only)")
        if not px4.arm_vehicle():
            return False
    else:
        print("[MISSION1 MOVE] Waiting for manual arm")
        if not px4.wait_for_arm_with_heartbeat(
            timeout=arm_timeout, heartbeat_rate=SETPOINT_RATE_HZ
        ):
            return False

    launch_pos = px4.get_location()
    if not launch_pos:
        print("[MISSION1 MOVE] Cannot get launch position")
        return False
    _launch_alt = launch_pos["z"]

    print(f"[MISSION1 MOVE] Taking off to {takeoff_altitude:g}m")
    if not px4.takeoff(altitude=takeoff_altitude, timeout=30):
        return False

    hover_pos = px4.get_location()
    if not hover_pos:
        print("[MISSION1 MOVE] Cannot get hover position")
        return False
    _hover_alt = hover_pos["z"]
    return True


def navigate_to_coordinate(
    lat: float,
    lon: float,
    alt_agl: float | None = None,
    timeout: float = 60.0,
    interrupt_check: Callable[[], str | None] | None = None,
    monitor_offboard: bool = False,
) -> bool | str:
    """Fly to a GPS lat/lon while holding current hover altitude by default."""
    px4 = connect()
    current_pos = px4.get_location()
    current_gps = px4.get_gps_location()
    if not current_pos or not current_gps:
        print("[MISSION1 MOVE] Cannot get current local/GPS position")
        return False

    east, north = gps_to_local_offset(
        current_gps["latitude"],
        current_gps["longitude"],
        float(lat),
        float(lon),
    )
    target_x = current_pos["x"] + east
    target_y = current_pos["y"] + north
    target_alt = (
        _hover_alt
        if alt_agl is None
        else (_launch_alt or current_pos["z"]) + float(alt_agl)
    )
    if target_alt is None:
        target_alt = current_pos["z"]

    print(
        f"[MISSION1 MOVE] GPS target lat={float(lat):.7f}, lon={float(lon):.7f}; "
        f"offset E/N=({east:.2f}, {north:.2f})m; local=({target_x:.2f}, {target_y:.2f}, {target_alt:.2f})"
    )

    dt = 1.0 / SETPOINT_RATE_HZ
    start_time = time.time()
    last_log = 0.0
    while (time.time() - start_time) < timeout:
        if interrupt_check is not None:
            action = interrupt_check()
            if action in {"p", "q"}:
                px4.hold_current_position()
                return "paused" if action == "p" else "stopped"

        current_pos = px4.get_location()
        if not current_pos:
            time.sleep(dt)
            continue

        if monitor_offboard:
            mode = px4.get_mode()
            if mode and mode != "OFFBOARD":
                print(
                    f"[MISSION1 MOVE] Vehicle left OFFBOARD mode ({mode}); pausing mission control"
                )
                return "manual"

        horizontal_distance = math.hypot(
            current_pos["x"] - target_x, current_pos["y"] - target_y
        )
        altitude_error = abs(current_pos["z"] - target_alt)
        px4.send_position_setpoint(
            target_x, target_y, target_alt, yaw=None, yaw_from_direction=True
        )

        now = time.time()
        if now - last_log >= 1.0:
            gps = px4.get_gps_location()
            if gps:
                print(
                    f"gps coordinate: lat={gps['latitude']:.7f}, lon={gps['longitude']:.7f}"
                )
            print(
                f"[MISSION1 MOVE] Distance XY: {horizontal_distance:.2f}m, alt error: {altitude_error:.2f}m"
            )
            last_log = now

        if (
            horizontal_distance <= ARRIVAL_TOLERANCE_M
            and altitude_error <= ALTITUDE_TOLERANCE_M
        ):
            return True
        time.sleep(dt)

    print("[MISSION1 MOVE] Timeout reaching GPS target")
    return False


def end_phase(*, land: bool = True) -> bool:
    px4 = connect()
    ok = True
    if land:
        print("[MISSION1 MOVE] Landing")
        ok = px4.land(timeout=60)
    px4.stop_offboard_stream_background()
    return ok


def cleanup(*, stop_agent_process: bool = False) -> None:
    """Shut down the PX4 interface in this process; optionally stop the agent this process started."""
    global _px4

    try:
        from drone.px4.interface import shutdown_px4

        shutdown_px4()
    except Exception:
        pass
    _px4 = None

    if stop_agent_process:
        try:
            from drone.px4.agent import stop_agent

            stop_agent()
        except Exception:
            pass
