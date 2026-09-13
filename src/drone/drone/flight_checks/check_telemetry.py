"""
Telemetry check: print everything the interface reads from PX4. Nothing is commanded.

Replaces drone-2026 mission_controller/mavros_testspace.py. Sanity checks for the
ENU convention while it runs:
  - lift the drone:          z (Up) increases
  - carry it north:          y (North) increases
  - point the nose north:    yaw is about +90°; nose east is about 0°

Example:
    ros2 run drone check_telemetry --sitl --duration 30
"""

import math
import time

from drone.flight_checks.common import build_parser, get_jetson_ip, run_main


def print_basic_telemetry(px4):
    print("[TELEMETRY] Basic Status")
    print(f"Connected:     {px4.connected}")
    print(f"Armed:         {px4.is_armed()}")
    print(f"Mode:          {px4.get_mode()}")

    loc = px4.get_location()
    if loc:
        print(
            f"Position (ENU): E={loc['x']:.2f}m, N={loc['y']:.2f}m, U={loc['z']:.2f}m"
        )
    else:
        print("Position (ENU): <not available>")
    print(f"Yaw (ENU):     {math.degrees(px4.get_current_yaw()):.1f}°")

    battery = px4.get_battery_status()
    if battery and battery["percentage"] is not None:
        print(
            f"Battery:       {battery['percentage'] * 100:.0f}% ({battery['voltage']:.2f}V, {battery['current']:.2f}A)"
        )
    elif battery:
        print(
            f"Battery:       unknown % ({battery['voltage']:.2f}V, {battery['current']:.2f}A)"
        )
    else:
        print("Battery:       <not available>")

    gps = px4.get_gps_location()
    if gps:
        print(
            f"GPS Position:  Lat={gps['latitude']:.6f}°, Lon={gps['longitude']:.6f}°, Alt(AMSL)={gps['altitude']}"
        )
    else:
        print("GPS Position:  <not available>")

    gps_raw = px4.get_gps_raw()
    if gps_raw:
        print(
            f"GPS Receiver:  fix_type={gps_raw['fix_type']}, satellites={gps_raw['satellites_visible']}"
        )

    velocity = px4.get_velocity()
    if velocity:
        print(
            f"Velocity (ENU): E={velocity['x']:.2f}, N={velocity['y']:.2f}, U={velocity['z']:.2f} m/s"
        )
    else:
        print("Velocity (ENU): <not available>")

    home = px4.get_home_location()
    if home:
        print(
            f"Home Position: Lat={home['latitude']:.6f}°, Lon={home['longitude']:.6f}°, Alt={home['altitude']}"
        )
    else:
        print("Home Position: <not available>")

    print(f"Landed:        {px4.is_landed()}")
    print("=" * 60)


def check(px4, args):
    print(f"[MAIN] Collecting telemetry for {args.duration:.0f} seconds")
    start = time.time()
    next_print = 0.0
    while (time.time() - start) < args.duration:
        if time.time() >= next_print:
            px4.print_telemetry_health()
            print_basic_telemetry(px4)
            next_print = time.time() + 2.0
        time.sleep(0.1)

    print("[TELEMETRY] Full Snapshot")
    for key, value in px4.get_full_telemetry_snapshot().items():
        print(f"{key}: {value}")
    return px4.connected


def main():
    parser = build_parser(__doc__)
    parser.add_argument(
        "--duration", type=float, default=30.0, help="Seconds to collect (default 30)"
    )
    args = parser.parse_args()
    print("=" * 60)
    print(f"[JETSON INFO] IP Address: {get_jetson_ip()}")
    print("=" * 60)
    return run_main(check, args)


if __name__ == "__main__":
    raise SystemExit(main())
