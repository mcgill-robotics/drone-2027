"""
Reading data from PX4: position, attitude, GPS, battery, mode, armed state.

Replaces drone-2026's px4_getters.py, and the getters return the same dict keys.

How it works:
  1. We subscribe to each PX4 topic in TELEMETRY_STREAMS.
  2. Whenever a message arrives, a small callback saves it in self._latest.
     This happens on the background thread from interface.py.
  3. When your code calls a getter such as get_location(), it takes the saved
     message and converts it to a simple ENU dict (see convert_ned_enu.py).

So getters never wait: they return whatever arrived most recently, or None if
nothing has arrived yet.
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

from drone.px4 import convert_ned_enu
from drone.px4.ned_enu_math_convert import euler_ned_to_enu
from drone.px4.modes import nav_state_names, normalize_mode_name
from drone.px4.qos import PX4_QOS
from drone.px4.topics import out_topic

# If PX4's status message has not arrived for this many seconds, treat PX4 as disconnected.
# Tune after measuring `ros2 topic hz` on the status topic in SITL and on the drone.
CONNECTION_TIMEOUT_S = 2.0

# Every PX4 topic we listen to, as (name we store it under, message type, topic name).
# PX4 only sends topics listed in its dds_topics.yaml; all of these are in PX4 1.17's.
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
    """
    The reading-data part of PX4Interface.

    Not used on its own: PX4Interface combines it with the other mixins, and it
    relies on PX4Interface being a ROS node with `namespace` set.
    """

    def _init_telemetry(self):
        """Subscribe to every topic in TELEMETRY_STREAMS."""
        self._latest = {key: None for key, _, _ in TELEMETRY_STREAMS}  # newest message
        self._received_at = {
            key: None for key, _, _ in TELEMETRY_STREAMS
        }  # when it came
        self._nav_state_names = nav_state_names(VehicleStatus)  # mode number -> name
        self._telemetry_subs = []

        for key, msg_type, base_name in TELEMETRY_STREAMS:
            topic = out_topic(msg_type, base_name, self.namespace)
            sub = self.create_subscription(
                msg_type, topic, self._make_store_callback(key), PX4_QOS
            )
            self._telemetry_subs.append(sub)
            print(f"[PX4] Subscribed to {topic}")

    def _make_store_callback(self, key):
        """Make the callback for one topic: it just saves the message and the time."""

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
        """
        True if PX4's status message arrived in the last CONNECTION_TIMEOUT_S seconds.

        Replaces MAVROS's State.connected.
        """
        received = self._received_at["status"]
        return (
            received is not None
            and (time.monotonic() - received) < CONNECTION_TIMEOUT_S
        )

    def wait_for_connection(self, timeout=30.0):
        """Wait until PX4's status message arrives. Returns False after `timeout` seconds."""
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
        """Same as wait_for_connection()."""
        return self.wait_for_connection(timeout)

    # =========================================================
    # State and mode
    # =========================================================

    def is_armed(self):
        """True if the motors are armed. False if not, or if PX4 is disconnected."""
        status = self._latest["status"]
        if not self.connected or status is None:
            return False
        return status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def get_mode(self):
        """Current flight mode as a name, e.g. "OFFBOARD", "POSCTL", "AUTO.RTL". None if unknown."""
        status = self._latest["status"]
        if status is None:
            return None
        nav_state = int(status.nav_state)
        return self._nav_state_names.get(nav_state, f"UNKNOWN({nav_state})")

    def wait_for_mode(self, mode, timeout=5.0):
        """
        Wait until PX4 reports it is actually in `mode`. Returns False after `timeout`.

        Use this after change_mode(): PX4 accepting the request does not always mean
        it switched.
        """
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
        """True if PX4 detects the drone is on the ground."""
        land = self._latest["land_detected"]
        return bool(land.landed) if land is not None else False

    # =========================================================
    # Position, velocity, attitude
    # All ENU (x East, y North, z Up) unless the method name says NED.
    # =========================================================

    def get_location(self):
        """
        Position {"x": east, "y": north, "z": up} in metres, or None.

        Measured from PX4's local origin, which is roughly where PX4 started up.
        """
        msg = self._latest["local_position"]
        return convert_ned_enu.local_position_enu(msg) if msg is not None else None

    def get_altitude(self):
        """Height above the local origin in metres (0 if unknown)."""
        loc = self.get_location()
        return loc["z"] if loc else 0

    def get_velocity(self):
        """Velocity {"x": east, "y": north, "z": up} in m/s, or None."""
        msg = self._latest["local_position"]
        return convert_ned_enu.local_velocity_enu(msg) if msg is not None else None

    def get_current_yaw(self):
        """
        Which way the nose points, in radians: 0 = East, pi/2 = North (counter-clockwise).

        Returns 0.0 if unknown.
        """
        msg = self._latest["local_position"]
        yaw = convert_ned_enu.heading_enu(msg) if msg is not None else None
        if yaw is None:
            attitude = self.get_attitude_enu()
            yaw = attitude[2] if attitude else None
        return yaw if yaw is not None else 0.0

    def get_attitude_ned(self):
        """(roll, pitch, yaw) in radians in PX4's own convention (NED), or None."""
        msg = self._latest["attitude"]
        return convert_ned_enu.attitude_ned(msg) if msg is not None else None

    def get_attitude_enu(self):
        """(roll, pitch, yaw) in radians in our convention (ENU), or None."""
        ned = self.get_attitude_ned()
        return euler_ned_to_enu(*ned) if ned else None

    def get_pose(self):
        """Position and attitude together in one dict, or None if neither is known."""
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
        """
        GPS position {"latitude", "longitude", "altitude"}, or None.

        This is PX4's best estimate, combining GPS with its other sensors.
        Altitude is metres above sea level.
        """
        msg = self._latest["global_position"]
        return convert_ned_enu.global_position(msg) if msg is not None else None

    def get_gps_raw(self, gps_id=1):
        """
        Raw data straight from the GPS receiver (fix type, satellites, accuracy).

        PX4 only sends one GPS, so gps_id=2 always returns None.
        """
        msg = self._latest["gps"]
        if gps_id != 1 or msg is None:
            return None
        return convert_ned_enu.gps_raw(msg)

    def get_home_location(self):
        """Where PX4 set home (usually where it armed), as latitude/longitude/altitude."""
        msg = self._latest["home"]
        return convert_ned_enu.home_position(msg) if msg is not None else None

    def get_battery_status(self):
        """{"voltage", "current", "percentage", "remaining"}, or None."""
        msg = self._latest["battery"]
        return convert_ned_enu.battery(msg) if msg is not None else None

    # =========================================================
    # Debug output
    # =========================================================

    def get_full_telemetry_snapshot(self):
        """Every getter's result in one dict, for printing or logging."""
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
        """Print the current state, position, velocity and GPS in a readable block."""
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
        """
        Print which PX4 topics are arriving and how fresh they are.

        A wrong topic name or QoS gives no error, just no data, so check this first
        when a getter keeps returning None.
        """
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
