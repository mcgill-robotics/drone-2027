"""
Telemetry from PX4: subscriptions and getters.

Replaces drone-2026's px4_getters.py. Callbacks only store the newest message and
when it arrived; getters convert on demand and return ENU dicts with the same keys
as before. Callbacks run on PX4Interface's executor thread, getters on the caller's
thread, so there is never a reason for callers to spin.
"""

import time

from px4_msgs.msg import (
    BatteryStatus,
    HomePosition,
    SensorGps,
    VehicleAttitude,
    VehicleGlobalPosition,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from drone.px4 import convert
from drone.px4.frames import euler_ned_to_enu
from drone.px4.modes import nav_state_names, normalize_mode_name
from drone.px4.qos import PX4_QOS
from drone.px4.topics import out_topic

# If no vehicle_status arrives for this long, treat the link to PX4 as down.
# Tune after measuring `ros2 topic hz` on the status topic in SITL and on the drone.
CONNECTION_TIMEOUT_S = 2.0

# (key, message type, topic base name). Every one of these is in PX4 1.16's dds_topics.yaml.
TELEMETRY_STREAMS = (
    ("status", VehicleStatus, "vehicle_status"),
    ("local_position", VehicleLocalPosition, "vehicle_local_position"),
    ("attitude", VehicleAttitude, "vehicle_attitude"),
    ("global_position", VehicleGlobalPosition, "vehicle_global_position"),
    ("gps", SensorGps, "vehicle_gps_position"),
    ("battery", BatteryStatus, "battery_status"),
    ("land_detected", VehicleLandDetected, "vehicle_land_detected"),
    ("home", HomePosition, "home_position"),
)


class TelemetryMixin:
    """Subscriptions + getters. Expects the host class to be a rclpy Node with `namespace` set."""

    def _init_telemetry(self):
        self._latest = {key: None for key, _, _ in TELEMETRY_STREAMS}
        self._received_at = {key: None for key, _, _ in TELEMETRY_STREAMS}
        self._nav_state_names = nav_state_names(VehicleStatus)
        self._telemetry_subs = []

        for key, msg_type, base_name in TELEMETRY_STREAMS:
            topic = out_topic(msg_type, base_name, self.namespace)
            sub = self.create_subscription(
                msg_type, topic, self._make_store_callback(key), PX4_QOS
            )
            self._telemetry_subs.append(sub)
            print(f"[PX4] Subscribed to {topic}")

    def _make_store_callback(self, key):
        def _store(msg):
            if self._latest[key] is None:
                print(f"[PX4] Receiving {key}")
            self._latest[key] = msg
            self._received_at[key] = time.monotonic()

        return _store

    # =========================================================
    # Connection
    # =========================================================

    @property
    def connected(self):
        """True while vehicle_status keeps arriving. Replaces MAVROS's State.connected."""
        received = self._received_at["status"]
        return (
            received is not None
            and (time.monotonic() - received) < CONNECTION_TIMEOUT_S
        )

    def wait_for_connection(self, timeout=30.0):
        """Block until PX4 telemetry arrives through the agent."""
        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            if self.connected:
                print("[PX4] Connected to PX4 (vehicle_status received)")
                return True
            time.sleep(0.1)
        print(
            f"[PX4] No vehicle_status within {timeout:.0f}s. Is MicroXRCEAgent running and PX4 connected to it?"
        )
        return False

    def connect(self, timeout=30.0):
        return self.wait_for_connection(timeout)

    # =========================================================
    # State and mode
    # =========================================================

    def is_armed(self):
        status = self._latest["status"]
        if not self.connected or status is None:
            return False
        return status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def get_mode(self):
        """Current flight mode as a MAVROS-style name ("OFFBOARD", "POSCTL", "AUTO.RTL", ...) or None."""
        status = self._latest["status"]
        if status is None:
            return None
        nav_state = int(status.nav_state)
        return self._nav_state_names.get(nav_state, f"UNKNOWN({nav_state})")

    def wait_for_mode(self, mode, timeout=5.0):
        """Block until PX4 reports `mode` as active (a command being accepted is not enough)."""
        target = normalize_mode_name(mode)
        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            if self.get_mode() == target:
                return True
            time.sleep(0.1)
        print(
            f"[PX4] Mode {target} not active after {timeout:.0f}s (current: {self.get_mode()})"
        )
        return False

    def is_landed(self):
        land = self._latest["land_detected"]
        return bool(land.landed) if land is not None else False

    # =========================================================
    # Position, velocity, attitude (ENU unless the name says NED)
    # =========================================================

    def get_location(self):
        """Local position {"x": east, "y": north, "z": up} in metres, or None."""
        msg = self._latest["local_position"]
        return convert.local_position_enu(msg) if msg is not None else None

    def get_altitude(self):
        loc = self.get_location()
        return loc["z"] if loc else 0

    def get_velocity(self):
        """Local velocity {"x": east, "y": north, "z": up} in m/s, or None."""
        msg = self._latest["local_position"]
        return convert.local_velocity_enu(msg) if msg is not None else None

    def get_current_yaw(self):
        """ENU yaw in radians (0 = facing East, counter-clockwise positive). 0.0 if unknown."""
        msg = self._latest["local_position"]
        yaw = convert.heading_enu(msg) if msg is not None else None
        if yaw is None:
            attitude = self.get_attitude_enu()
            yaw = attitude[2] if attitude else None
        return yaw if yaw is not None else 0.0

    def get_attitude_ned(self):
        """(roll, pitch, yaw) in radians, FRD body relative to NED, or None. Used by api.geo."""
        msg = self._latest["attitude"]
        return convert.attitude_ned(msg) if msg is not None else None

    def get_attitude_enu(self):
        """(roll, pitch, yaw) in radians, FLU body relative to ENU, or None."""
        ned = self.get_attitude_ned()
        return euler_ned_to_enu(*ned) if ned else None

    def get_pose(self):
        position = self.get_location()
        attitude = self.get_attitude_enu()
        if position is None and attitude is None:
            return None
        return {
            "position": position,
            "orientation_euler_rad": (
                {"roll": attitude[0], "pitch": attitude[1], "yaw": attitude[2]}
                if attitude
                else None
            ),
        }

    # =========================================================
    # GPS, home, battery
    # =========================================================

    def get_gps_location(self):
        """Fused global position {"latitude", "longitude", "altitude" (AMSL m)} or None."""
        msg = self._latest["global_position"]
        return convert.global_position(msg) if msg is not None else None

    def get_gps_raw(self, gps_id=1):
        """Raw receiver data. PX4 bridges one GPS (vehicle_gps_position), so gps_id 2 returns None."""
        msg = self._latest["gps"]
        if gps_id != 1 or msg is None:
            return None
        return convert.gps_raw(msg)

    def get_home_location(self):
        msg = self._latest["home"]
        return convert.home_position(msg) if msg is not None else None

    def get_battery_status(self):
        msg = self._latest["battery"]
        return convert.battery(msg) if msg is not None else None

    # =========================================================
    # Debug output
    # =========================================================

    def get_full_telemetry_snapshot(self):
        return {
            "status": {
                "connected": self.connected,
                "armed": self.is_armed(),
                "mode": self.get_mode(),
                "landed": self.is_landed(),
            },
            "battery": self.get_battery_status(),
            "home": self.get_home_location(),
            "gps_location": self.get_gps_location(),
            "gps_raw": self.get_gps_raw(),
            "pose": self.get_pose(),
            "velocity": self.get_velocity(),
            "attitude_ned_rad": self.get_attitude_ned(),
        }

    def print_telemetry_summary(self):
        snapshot = self.get_full_telemetry_snapshot()
        status = snapshot["status"]
        print("=" * 60)
        print("[TELEMETRY] Current Status")
        print(f"Connected: {status['connected']}")
        print(f"Armed:     {status['armed']}")
        print(f"Mode:      {status['mode']}")
        print(f"Landed:    {status['landed']}")

        pose = snapshot["pose"]
        if pose and pose["position"]:
            pos = pose["position"]
            print(
                f"Position (ENU): E={pos['x']:.2f}, N={pos['y']:.2f}, U={pos['z']:.2f}"
            )
        if pose and pose["orientation_euler_rad"]:
            ori = pose["orientation_euler_rad"]
            print(
                f"Orientation (ENU RPY rad): R={ori['roll']:.3f}, P={ori['pitch']:.3f}, Y={ori['yaw']:.3f}"
            )

        vel = snapshot["velocity"]
        if vel:
            print(
                f"Velocity (ENU): E={vel['x']:.2f}, N={vel['y']:.2f}, U={vel['z']:.2f}"
            )

        gps = snapshot["gps_location"]
        if gps:
            print(
                f"GPS Position: lat={gps['latitude']}, lon={gps['longitude']}, alt={gps['altitude']}"
            )

        gps_raw = snapshot["gps_raw"]
        if gps_raw:
            print(
                f"GPS Raw: fix_type={gps_raw['fix_type']}, satellites={gps_raw['satellites_visible']}, "
                f"eph={gps_raw['eph']}, epv={gps_raw['epv']}"
            )
        print("=" * 60)

    def print_telemetry_health(self, stale_after_sec=2.0):
        """Which PX4 topics are arriving. Wrong topic names or QoS fail silently, so check this first."""
        now = time.monotonic()
        print("=" * 60)
        print("[PX4] Telemetry Health Check")
        for key, msg_type, base_name in TELEMETRY_STREAMS:
            received = self._received_at[key]
            if received is None:
                state = "NO DATA"
            elif now - received > stale_after_sec:
                state = f"STALE ({now - received:.1f}s old)"
            else:
                state = f"OK ({now - received:.1f}s old)"
            print(f"{out_topic(msg_type, base_name, self.namespace):<42} {state}")
        print("=" * 60)
