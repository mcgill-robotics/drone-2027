"""
GPS movement check: OFFBOARD, arm, take off 5 m, fly to a GPS target facing the direction of travel, land.

Replaces drone-2026 tests/test_gps_movement.py. That script set PX4 speed-limit
parameters from code; the bridge cannot do that, so set them in QGroundControl first
(docs/px4_setup.md): MPC_XY_VEL_MAX 2.0, MPC_XY_CRUISE 1.0, MPC_VEL_MANUAL 2.0,
MPC_LAND_SPEED 0.4, MPC_LAND_CRWL 0.2.

Target altitude is metres above the launch point, not AMSL. Omit it to hold the
post-takeoff altitude.

Examples:
    ros2 run drone check_gps_movement --sitl --api --target 47.39785,8.54573
    ros2 run drone check_gps_movement --serial --target 45.5065,-73.5803,6
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

ARRIVAL_TOLERANCE_M = 0.5
ALTITUDE_TOLERANCE_M = 0.5
SETPOINT_RATE_HZ = 10
TAKEOFF_ALTITUDE_M = 5.0


def fly_to_target(px4, target_gps, launch_alt, hover_alt):
    target_lat, target_lon, target_z = target_gps
    current_pos = px4.get_location()
    current_gps = px4.get_gps_location()
    if not current_pos or not current_gps:
        print("[CHECK] Cannot get position or GPS location")
        return False

    east, north = gps_to_local_offset(
        current_gps["latitude"], current_gps["longitude"], target_lat, target_lon
    )
    target_x = current_pos["x"] + east
    target_y = current_pos["y"] + north
    target_alt = hover_alt if target_z is None else launch_alt + target_z
    print(
        f"\n[CHECK] Flying to target GPS: lat={target_lat:.6f}, lon={target_lon:.6f}, alt={target_alt:.2f}m local"
    )
    print(f"[CHECK] Offset (E,N): ({east:.2f}m, {north:.2f}m)")
    print(
        f"[CHECK] Local target (ENU): x={target_x:.2f}, y={target_y:.2f}, z={target_alt:.2f}"
    )

    dt = 1.0 / SETPOINT_RATE_HZ
    start = time.time()
    last_log = 0.0
    while (time.time() - start) < 60:
        current_pos = px4.get_location()
        if not current_pos:
            time.sleep(dt)
            continue

        horizontal_distance = math.hypot(
            current_pos["x"] - target_x, current_pos["y"] - target_y
        )
        altitude_error = abs(current_pos["z"] - target_alt)
        px4.send_position_setpoint(
            target_x, target_y, target_alt, yaw=None, yaw_from_direction=True
        )

        now = time.time()
        if now - last_log >= 1.0:
            print(
                f"[CHECK] Distance XY: {horizontal_distance:.2f}m, alt error: {altitude_error:.2f}m"
            )
            last_log = now
        if (
            horizontal_distance <= ARRIVAL_TOLERANCE_M
            and altitude_error <= ALTITUDE_TOLERANCE_M
        ):
            print(
                f"[CHECK] ✓ Reached target (xy={horizontal_distance:.2f}m, alt_err={altitude_error:.2f}m)"
            )
            return True
        time.sleep(dt)

    print("[CHECK] Timeout reaching target")
    return False


def check(px4, args):
    print(
        "[CHECK] Reminder: motion limits (MPC_XY_VEL_MAX etc.) must already be set in QGC"
    )
    if not enter_offboard(px4):
        return False
    if not arm(px4, args):
        print("[CHECK] Vehicle was not armed")
        return False

    launch_pos = px4.get_location()
    if not launch_pos:
        print("[CHECK] Cannot get launch position before takeoff")
        return False

    print(f"\n[CHECK] Taking off to {TAKEOFF_ALTITUDE_M:.0f} meters...")
    if not px4.takeoff(altitude=TAKEOFF_ALTITUDE_M, timeout=30):
        print("[CHECK] Takeoff failed")
        return False

    hover_pos = px4.get_location()
    if not hover_pos:
        print("[CHECK] Cannot get hover position after takeoff")
        return False

    if args.target is None:
        print("\n[CHECK] No target GPS provided. Hovering for 10 seconds...")
        time.sleep(10)
    elif not fly_to_target(px4, args.target, launch_pos["z"], hover_pos["z"]):
        return False

    print("\n[CHECK] Landing...")
    if not px4.land(timeout=60):
        print("[CHECK] Landing failed")
        return False
    return px4.stop_offboard_stream_background()


def parse_target(text):
    parts = text.split(",")
    if len(parts) not in (2, 3):
        raise ValueError("expected lat,lon or lat,lon,alt")
    alt = float(parts[2]) if len(parts) == 3 else None
    return float(parts[0]), float(parts[1]), alt


def main():
    parser = build_parser(__doc__, api_arm=True)
    parser.add_argument(
        "--target", default=None, help='Target as "lat,lon[,alt_above_launch]"'
    )
    args = parser.parse_args()
    if not api_arm_allowed(args):
        return 2
    if args.target is not None:
        try:
            args.target = parse_target(args.target)
        except ValueError:
            print("[MAIN] Invalid target format. Use: 'lat,lon' or 'lat,lon,alt_agl'")
            return 2
    return run_main(check, args)


if __name__ == "__main__":
    raise SystemExit(main())
