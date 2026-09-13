"""
GPS goto check: take off, fly to one GPS coordinate with absolute position setpoints, land.

Replaces drone-2026 tests/test_goto_gps.py. That script switched to GUIDED, an
ArduPilot mode PX4 does not have; this one stays in OFFBOARD the whole time.

The setpoint published every loop is always the same absolute local target, so as the
drone closes in the remaining distance shrinks on its own.

Example (PX4 SITL's default spawn is near 47.3977, 8.5456):
    ros2 run drone check_goto_gps --sitl --api --lat 47.39785 --lon 8.54573 --alt 10
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

ARRIVAL_TOLERANCE_M = 1.0
SETPOINT_RATE_HZ = 10
GOTO_TIMEOUT_S = 120


def resolve_target_local(px4, target_lat, target_lon, target_alt):
    """Anchor the GPS target to the drone's current ENU local frame."""
    gps = px4.get_gps_location()
    loc = px4.get_location()
    if gps is None or loc is None:
        print("[CHECK] Missing GPS or local position; cannot resolve target")
        return None

    east, north = gps_to_local_offset(
        gps["latitude"], gps["longitude"], target_lat, target_lon
    )
    target = (loc["x"] + east, loc["y"] + north, target_alt)
    print(f"[CHECK] Current GPS: ({gps['latitude']:.7f}, {gps['longitude']:.7f})")
    print(f"[CHECK] Target  GPS: ({target_lat:.7f}, {target_lon:.7f})")
    print(f"[CHECK] Offset (E,N): ({east:.2f} m, {north:.2f} m)")
    print(
        f"[CHECK] Absolute local target (ENU): x={target[0]:.2f}, y={target[1]:.2f}, z={target[2]:.2f}"
    )
    return target


def distance_to(px4, tx, ty, tz):
    loc = px4.get_location()
    if loc is None:
        return float("inf")
    return math.sqrt((loc["x"] - tx) ** 2 + (loc["y"] - ty) ** 2 + (loc["z"] - tz) ** 2)


def check(px4, args):
    if not enter_offboard(px4):
        return False
    if not arm(px4, args):
        print("[CHECK] Vehicle was not armed")
        return False

    print(f"[CHECK] Taking off to {args.alt:.1f} m...")
    if not px4.takeoff(args.alt, timeout=60):
        print("[CHECK] Takeoff failed")
        return False

    print("[CHECK] Waiting for telemetry to stabilize...")
    settle_start = time.time()
    while time.time() - settle_start < 5 and not (
        px4.get_gps_location() and px4.get_location()
    ):
        time.sleep(0.1)

    target = resolve_target_local(px4, args.lat, args.lon, args.alt)
    if target is None:
        return False
    tx, ty, tz = target

    print(f"[CHECK] Streaming absolute setpoints at {SETPOINT_RATE_HZ} Hz...")
    dt = 1.0 / SETPOINT_RATE_HZ
    start = time.time()
    last_log = 0.0
    while True:
        px4.send_position_setpoint(tx, ty, tz)
        dist = distance_to(px4, tx, ty, tz)
        now = time.time()
        if now - last_log >= 1.0:
            print(f"[CHECK] Distance to target: {dist:.2f} m")
            last_log = now
        if dist <= ARRIVAL_TOLERANCE_M:
            print(f"[CHECK] ✓ Arrived (within {ARRIVAL_TOLERANCE_M:.1f} m)")
            break
        if now - start > GOTO_TIMEOUT_S:
            print(f"[CHECK] ✗ Timeout after {GOTO_TIMEOUT_S}s, dist={dist:.2f} m")
            return False
        time.sleep(dt)

    # Hold the target briefly to confirm stability (heartbeat keeps republishing it).
    time.sleep(3.0)

    print("[CHECK] Landing...")
    return px4.land(timeout=60)


def main():
    parser = build_parser(__doc__, api_arm=True)
    parser.add_argument(
        "--lat", type=float, required=True, help="Target latitude (deg)"
    )
    parser.add_argument(
        "--lon", type=float, required=True, help="Target longitude (deg)"
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=10.0,
        help="Takeoff climb and target local altitude (m)",
    )
    args = parser.parse_args()
    if not api_arm_allowed(args):
        return 2
    return run_main(check, args)


if __name__ == "__main__":
    raise SystemExit(main())
