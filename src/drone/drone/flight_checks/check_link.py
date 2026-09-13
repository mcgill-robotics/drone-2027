"""
Link check: start the agent and confirm PX4 is publishing into ROS 2.

Replaces drone-2026 tests/test_boot.py, which launched MAVROS and looked for /mavros topics.
Counts publishers rather than listing topic names, because our own subscriptions make a
topic name appear even when PX4 is not sending anything.

Examples:
    ros2 run drone check_link --sitl
    ros2 run drone check_link --serial /dev/ttyTHS1
    ros2 run drone check_link --udp 8888
"""

from drone.flight_checks.common import build_parser, run_main
from drone.px4.telemetry import TELEMETRY_STREAMS
from drone.px4.topics import out_topic

REQUIRED = ("status", "local_position", "battery")


def check(px4, args):
    missing = []
    print("\n[CHECK] PX4 publishers visible through the agent:")
    for key, msg_type, base_name in TELEMETRY_STREAMS:
        topic = out_topic(msg_type, base_name, px4.namespace)
        count = px4.count_publishers(topic)
        print(f"    {topic:<42} publishers={count}")
        if key in REQUIRED and count == 0:
            missing.append(topic)

    px4.print_telemetry_health()
    if missing:
        print(f"[CHECK] Nothing publishing on: {', '.join(missing)}")
    return px4.connected and not missing


def main():
    return run_main(check, build_parser(__doc__).parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
