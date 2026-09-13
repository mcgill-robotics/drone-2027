"""
GPS lap check: take off, follow GPS waypoints with smooth velocity control, return to launch.

Replaces drone-2026 tests/test_lap.py, which no longer parsed (unterminated string).
Behaviour kept from it:
  - take off to --alt and record the actual hover altitude
  - fly each waypoint with velocity setpoints that slow down near the target, holding that altitude
  - switch to RTL after the last waypoint, or on any failure

Edit LAP_WAYPOINTS below for the field. Example:
    ros2 run drone check_lap --sitl --api
"""

import math
import time

from drone.flight_checks.common import (
    api_arm_allowed,
    arm,
    build_parser,
    enter_offboard,
    gps_to_local_offset,
    run_main,
)

# Format: (latitude, longitude). Altitude is held automatically.
LAP_WAYPOINTS = [
    (45.504800, -73.577200),
    (45.504850, -73.577250),
    (45.504900, -73.577200),
    (45.504850, -73.577150),
]

ARRIVAL_TOLERANCE_M = 2.0
ALTITUDE_TOLERANCE_M = 0.8
WAYPOINT_TIMEOUT_S = 90
POST_RTL_HOLD_S = 5
CONTROL_RATE_HZ = 10
MAX_XY_SPEED_MPS = 1.0
MIN_XY_SPEED_MPS = 0.15
MAX_Z_SPEED_MPS = 0.4
SLOWDOWN_RADIUS_M = 6.0


def clamp(value, low, high):
    return max(low, min(high, value))


def resolve_waypoint(px4, lat, lon, mission_alt):
    current_gps = px4.get_gps_location()
    current_pos = px4.get_location()
    if not current_gps or not current_pos:
        print("[CHECK] Missing GPS or local position")
        return None

    east, north = gps_to_local_offset(
        current_gps["latitude"], current_gps["longitude"], lat, lon
    )
    target = (current_pos["x"] + east, current_pos["y"] + north, mission_alt)
    print(f"[CHECK] GPS target: lat={lat:.7f}, lon={lon:.7f}")
    print(f"[CHECK] Offset: east={east:.2f}m, north={north:.2f}m")
    print(
        f"[CHECK] Local target (ENU): x={target[0]:.2f}, y={target[1]:.2f}, z={target[2]:.2f}"
    )
    return target


def go_to_local_smooth(px4, target_x, target_y, target_z, index):
    """Velocity control toward one ENU target: fast when far, slow when near, stop on arrival."""
    dt = 1.0 / CONTROL_RATE_HZ
    start = time.time()
    last_log = 0.0

    while time.time() - start < WAYPOINT_TIMEOUT_S:
        pos = px4.get_location()
        if not pos:
            time.sleep(dt)
            continue

        dx = target_x - pos["x"]
        dy = target_y - pos["y"]
        dz = target_z - pos["z"]
        xy_dist = math.hypot(dx, dy)

        if xy_dist <= ARRIVAL_TOLERANCE_M and abs(dz) <= ALTITUDE_TOLERANCE_M:
            px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            print(f"[CHECK] ✓ Reached waypoint {index}")
            return True

        xy_speed = clamp(
            MAX_XY_SPEED_MPS * (xy_dist / SLOWDOWN_RADIUS_M),
            MIN_XY_SPEED_MPS,
            MAX_XY_SPEED_MPS,
        )
        vx = xy_speed * dx / xy_dist if xy_dist > 0.05 else 0.0
        vy = xy_speed * dy / xy_dist if xy_dist > 0.05 else 0.0
        vz = clamp(0.5 * dz, -MAX_Z_SPEED_MPS, MAX_Z_SPEED_MPS)
        px4.send_velocity_setpoint(vx, vy, vz, 0.0)

        now = time.time()
        if now - last_log >= 1.0:
            print(
                f"[CHECK] WP{index}: xy={xy_dist:.2f}m, alt_err={abs(dz):.2f}m, vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}"
            )
            last_log = now
        time.sleep(dt)

    px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
    print(f"[CHECK] ✗ Timeout at waypoint {index}")
    return False


def rtl(px4):
    print("[CHECK] Switching to RTL...")
    ok = px4.change_mode("RTL")
    time.sleep(POST_RTL_HOLD_S)
    return ok


def check(px4, args):
    try:
        if not enter_offboard(px4):
            return False
        if not arm(px4, args):
            print("[CHECK] Arm timeout")
            return False

        print(f"[CHECK] Taking off to {args.alt:.1f}m...")
        if not px4.takeoff(altitude=args.alt, timeout=60):
            print("[CHECK] Takeoff failed")
            return False
        time.sleep(2)

        hover_pos = px4.get_location()
        if not hover_pos:
            print("[CHECK] Cannot read post-takeoff position")
            return False
        mission_alt = hover_pos["z"]
        print(f"[CHECK] Holding mission altitude: {mission_alt:.2f}m")

        for index, (lat, lon) in enumerate(LAP_WAYPOINTS, start=1):
            print("\n" + "-" * 60)
            print(f"[CHECK] Going to waypoint {index}")
            target = resolve_waypoint(px4, lat, lon, mission_alt)
            if target is None or not go_to_local_smooth(px4, *target, index=index):
                print("[CHECK] Waypoint failed. RTL for safety.")
                rtl(px4)
                return False

        print("\n[CHECK] All waypoints complete.")
        return rtl(px4)
    except Exception as e:
        print(f"[CHECK] Exception: {e}")
        rtl(px4)
        return False


def main():
    parser = build_parser(__doc__, api_arm=True)
    parser.add_argument(
        "--alt", type=float, default=5.0, help="Takeoff altitude in meters (default 5)"
    )
    args = parser.parse_args()
    if not api_arm_allowed(args):
        return 2
    return run_main(check, args)


if __name__ == "__main__":
    raise SystemExit(main())
