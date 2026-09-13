"""
OFFBOARD check: start the heartbeat, switch to OFFBOARD, confirm it holds, stop.

Replaces drone-2026 tests/test_offboard.py. Does not arm, so the drone does not move.

Example:
    ros2 run drone check_offboard --sitl
"""

import time

from drone.flight_checks.common import build_parser, enter_offboard, run_main


def check(px4, args):
    if not enter_offboard(px4):
        return False

    print(f"[CHECK] Drone armed: {px4.is_armed()}")
    print(
        f"[CHECK] Maintaining OFFBOARD for {args.hold:.0f} seconds (background heartbeat publishing)..."
    )
    end = time.time() + args.hold
    while time.time() < end:
        if px4.get_mode() != "OFFBOARD":
            print(f"[CHECK] OFFBOARD dropped to {px4.get_mode()}")
            return False
        time.sleep(0.2)
    print(f"[CHECK] ✓ OFFBOARD stable for {args.hold:.0f} seconds")

    return px4.stop_offboard_stream_background()


def main():
    parser = build_parser(__doc__)
    parser.add_argument(
        "--hold",
        type=float,
        default=5.0,
        help="Seconds to stay in OFFBOARD (default 5)",
    )
    return run_main(check, parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
