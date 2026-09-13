"""
Arming check in OFFBOARD: heartbeat, OFFBOARD, pre-arm diagnostics, then arm.

Replaces drone-2026 tests/test_arm.py. By default it waits for the pilot's RC arm
switch (docs/arming.md). --api arms from code; on the real drone only do that for a
props-off bench test. If this check armed the vehicle it also disarms it.

Examples:
    ros2 run drone check_arm --sitl --api
    ros2 run drone check_arm --serial          # then flip the RC arm switch
"""

import argparse
import time

from drone.flight_checks.common import enter_offboard, run_main
from drone.px4.agent import add_link_args


def print_diagnostics(px4):
    print("\n=== PRE-ARM DIAGNOSTICS ===")
    print(f"Armed: {px4.is_armed()}")
    battery = px4.get_battery_status()
    if battery:
        print(f"Battery Voltage: {battery['voltage']:.2f}V")
        print(f"Battery Current: {battery['current']:.2f}A")
        remaining = battery["remaining"]
        print(
            f"Battery Remaining: {remaining * 100:.0f}%"
            if remaining is not None
            else "Battery Remaining: unknown"
        )
    gps = px4.get_gps_raw()
    if gps:
        print(f"GPS Satellites: {gps['satellites_visible']}")
        print(f"GPS Fix: {gps['fix_type']}")
    home = px4.get_home_location()
    print(
        f"Home Set: Yes ({home['latitude']}, {home['longitude']}, {home['altitude']}m)"
        if home
        else "Home Set: No"
    )
    print("===========================\n")


def check(px4, args):
    # Heartbeat first, so OFFBOARD setpoints are already flowing during the
    # diagnostics and arm wait below (gaps longer than COM_OF_LOSS_T drop OFFBOARD).
    if not enter_offboard(px4):
        return False

    print_diagnostics(px4)

    if args.api:
        if not args.sitl:
            print("[CHECK] Arming from code on real hardware: props must be OFF")
        if not px4.arm_vehicle(timeout=20):
            print("[CHECK] API arm failed")
            return False
    else:
        print("[CHECK] Waiting for manual arm (60 seconds). Flip the RC arm switch.")
        start = time.time()
        while time.time() - start < 60:
            if px4.is_armed():
                break
            mode = px4.get_mode()
            if mode != "OFFBOARD":
                print(f"[CHECK] Mode dropped from OFFBOARD to {mode}; aborting")
                return False
            time.sleep(0.2)
        else:
            print("[CHECK] Arm timeout: vehicle was not armed")
            return False

    print("[CHECK] ✓ Vehicle armed")
    if args.api:
        px4.disarm_vehicle()
    else:
        print("[CHECK] Disarm with the RC switch now.")
    return True


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    add_link_args(parser)
    parser.add_argument(
        "--api",
        action="store_true",
        help="Arm from code instead of waiting for the RC switch",
    )
    return run_main(check, parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
