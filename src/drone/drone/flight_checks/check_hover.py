"""
Hover check: OFFBOARD, arm, take off, hover, land.

Replaces drone-2026 tests/test_hover.py. The pilot arms with the RC switch; --api
arms from code and is only allowed together with --sitl.

Examples:
    ros2 run drone check_hover --sitl --api
    ros2 run drone check_hover --serial --altitude 3
"""

import time

from drone.flight_checks.common import (
    api_arm_allowed,
    arm,
    build_parser,
    enter_offboard,
    run_main,
)


def check(px4, args):
    if not enter_offboard(px4):
        return False
    if not arm(px4, args):
        print("[CHECK] Vehicle was not armed")
        return False

    print(f"\n[CHECK] Taking off to {args.altitude:.1f} meters...")
    if not px4.takeoff(altitude=args.altitude, timeout=30):
        print("[CHECK] Takeoff failed")
        return False

    print(f"\n[CHECK] Hovering for {args.hover_seconds:.0f} seconds...")
    time.sleep(args.hover_seconds)

    print("\n[CHECK] Landing...")
    if not px4.land(timeout=30):
        print("[CHECK] Landing failed")
        return False
    return px4.stop_offboard_stream_background()


def main():
    parser = build_parser(__doc__, api_arm=True)
    parser.add_argument(
        "--altitude", type=float, default=5.0, help="Climb this many metres (default 5)"
    )
    parser.add_argument(
        "--hover-seconds", type=float, default=10.0, help="Hover time (default 10)"
    )
    args = parser.parse_args()
    if not api_arm_allowed(args):
        return 2
    return run_main(check, args)


if __name__ == "__main__":
    raise SystemExit(main())
