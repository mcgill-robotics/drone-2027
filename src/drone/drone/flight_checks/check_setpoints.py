"""
Setpoint check: publish offboard setpoints and request OFFBOARD, without arming.

Replaces drone-2026 mission_controller/mavros_setter_testspace.py and tests/test_commands.py.
The vehicle is never armed, so nothing moves. Watch the messages in another terminal:

    ros2 topic echo /fmu/in/trajectory_setpoint
    ros2 topic echo /fmu/in/offboard_control_mode

Values are given in ENU (x East, y North, z Up) and appear in NED on the topic
(x North, y East, z Down), which is itself a useful check of the conversion.

Example:
    ros2 run drone check_setpoints --sitl
"""

import time

from drone.flight_checks.common import build_parser, run_main


def stream_velocity(px4, label, vx, vy, vz, seconds=1.0, rate_hz=10):
    print(f"[CHECK] {label}: ENU velocity ({vx}, {vy}, {vz}) m/s for {seconds:.0f}s")
    count = 0
    end = time.time() + seconds
    while time.time() < end:
        if px4.send_velocity_setpoint(vx, vy, vz, 0.0):
            count += 1
        time.sleep(1.0 / rate_hz)
    print(f"[CHECK]   published {count} setpoints")
    return count > 0


def check(px4, args):
    if px4.is_armed():
        print("[CHECK] Vehicle is ARMED. Disarm before running this check.")
        return False

    ok = True
    loc = px4.get_location()
    if loc:
        sent = px4.send_position_setpoint(loc["x"], loc["y"], loc["z"])
        print(f"[CHECK] send_position_setpoint(current ENU position) -> {sent}")
        ok = ok and sent
    else:
        print("[CHECK] Local position unavailable; skipping position setpoint")

    ok = stream_velocity(px4, "East", 1.0, 0.0, 0.0) and ok
    ok = stream_velocity(px4, "West", -1.0, 0.0, 0.0) and ok
    ok = stream_velocity(px4, "North", 0.0, 1.0, 0.0) and ok
    ok = stream_velocity(px4, "Up (positive z is up in ENU)", 0.0, 0.0, 1.0) and ok
    ok = stream_velocity(px4, "Stop", 0.0, 0.0, 0.0) and ok
    if loc:
        ok = px4.hold_current_position() and ok

    print(f"[CHECK] Mode before OFFBOARD request: {px4.get_mode()}")
    px4.start_offboard_stream_background()
    accepted = px4.start_offboard()
    active = px4.wait_for_mode("OFFBOARD", timeout=5) if accepted else False
    print(
        f"[CHECK] OFFBOARD accepted={accepted}, active={active}, mode now: {px4.get_mode()}"
    )
    px4.stop_offboard_stream_background()
    return ok and accepted and active


def main():
    return run_main(check, build_parser(__doc__).parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
