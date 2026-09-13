"""
Shared plumbing for the flight checks.

Every check has the same shape:

    args = build_parser(__doc__).parse_args()
    try:
        px4 = connect(args)      # start MicroXRCEAgent (unless --no-agent), then init_px4()
        ...
    finally:
        shutdown()               # stop heartbeat, ROS 2 and the agent we started
"""

import argparse
import math
import subprocess

from drone.px4.agent import add_link_args, start_agent_from_args, stop_agent

EARTH_RADIUS_M = 6_378_137.0


def build_parser(description, api_arm=False):
    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    add_link_args(parser)
    if api_arm:
        parser.add_argument(
            "--api",
            action="store_true",
            help="Arm from code instead of waiting for the RC arm switch (only allowed with --sitl)",
        )
    return parser


def api_arm_allowed(args):
    """The pilot arms the real drone (docs/arming.md). Arming from code is for the simulator."""
    if getattr(args, "api", False) and not args.sitl:
        print(
            "[MAIN] --api arming is only allowed with --sitl. On the drone, the pilot arms with the RC switch."
        )
        return False
    return True


def connect(args, connect_timeout=30.0):
    """Start the agent for the chosen link and connect to PX4. Returns the PX4Interface or None."""
    from drone.px4.interface import init_px4

    if not start_agent_from_args(args):
        return None

    print("[MAIN] Connecting to PX4 through the agent...")
    px4 = init_px4(connect_timeout=connect_timeout)
    if not px4.connected:
        print("[MAIN] Failed to connect to PX4. Make sure:")
        print(
            "  1. PX4 SITL is running (make px4_sitl gz_x500) or the flight controller is powered"
        )
        print(
            "  2. The link flag matches how PX4 is configured (--sitl / --udp / --serial)"
        )
        print("  3. `ros2 topic list | grep fmu` shows /fmu/out topics")
        return None
    print("[MAIN] ✓ Connected to PX4")
    return px4


def shutdown():
    from drone.px4.interface import shutdown_px4

    shutdown_px4()
    stop_agent()


def enter_offboard(px4):
    """Start the heartbeat, switch to OFFBOARD, and confirm PX4 actually entered it."""
    px4.start_offboard_stream_background()
    if not px4.start_offboard():
        print("[CHECK] OFFBOARD request was not accepted")
        return False
    if not px4.wait_for_mode("OFFBOARD", timeout=5):
        print("[CHECK] OFFBOARD accepted but never became active")
        return False
    print("[CHECK] ✓ OFFBOARD active")
    return True


def arm(px4, args, timeout=60):
    """Arm from code (--api) or wait for the pilot's RC arm switch while streaming setpoints."""
    if getattr(args, "api", False):
        return px4.arm_vehicle(timeout=20)
    print("[CHECK] Arm the vehicle with the RC transmitter (or QGC)")
    return px4.wait_for_arm_with_heartbeat(timeout=timeout, heartbeat_rate=10)


def gps_to_local_offset(origin_lat, origin_lon, target_lat, target_lon):
    """Equirectangular GPS difference -> (east, north) metres. Good to centimetres over a field."""
    origin_lat_rad = math.radians(origin_lat)
    north = math.radians(target_lat - origin_lat) * EARTH_RADIUS_M
    east = (
        math.radians(target_lon - origin_lon)
        * EARTH_RADIUS_M
        * math.cos(origin_lat_rad)
    )
    return east, north


def get_jetson_ip():
    """First address from `hostname -I`, or "IP_NOT_FOUND"."""
    try:
        result = subprocess.run(
            ["hostname", "-I"], capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0 and result.stdout.split():
            return result.stdout.split()[0]
    except Exception as e:
        print(f"[ERROR] Failed to get Jetson IP: {str(e)}")
    return "IP_NOT_FOUND"


def run_main(check, args):
    """Connect, run `check(px4, args)`, always clean up. Returns a process exit code."""
    try:
        px4 = connect(args)
        if px4 is None:
            return 1
        ok = check(px4, args)
        print("\n[MAIN] ✓ Check passed" if ok else "\n[MAIN] ✗ Check failed")
        return 0 if ok else 1
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user")
        return 1
    finally:
        shutdown()
