"""
Offboard control: position/velocity setpoints and the heartbeat that keeps OFFBOARD alive.

Replaces the setpoint publishers and heartbeat thread in drone-2026's px4_setters.py.
Callers still pass ENU values. Every setpoint goes out as two messages:

    OffboardControlMode   which kind of control, and the "still alive" signal PX4 watches
    TrajectorySetpoint    the numbers, converted to NED by drone.px4.setpoints

PX4 leaves OFFBOARD if OffboardControlMode stops arriving at roughly 2 Hz or more,
so the background heartbeat republishes the last setpoint while mission code is busy.
"""

import math
import threading
import time

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

from drone.px4 import setpoints
from drone.px4.qos import PX4_QOS
from drone.px4.topics import in_topic

FT_TO_M = 0.3048
DEFAULT_MAX_ALT_FT = 400  # regulatory ceiling


class OffboardMixin:
    """Setpoint publishing. Expects a rclpy Node host with `namespace`, `_now_us()` and the other mixins."""

    def _init_offboard(self):
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            in_topic(OffboardControlMode, "offboard_control_mode", self.namespace),
            PX4_QOS,
        )
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            in_topic(TrajectorySetpoint, "trajectory_setpoint", self.namespace),
            PX4_QOS,
        )
        self._setpoint_lock = threading.Lock()
        self._last_setpoint = (
            None  # (kind, fields) most recently sent, for the heartbeat
        )
        self._last_user_publish_time = 0.0

        self._stream_thread = None
        self._stream_running = False
        self._stream_lock = threading.Lock()

        self._mission_yaw = None  # ENU yaw locked at takeoff
        self._max_alt_m = None  # software altitude clamp; None = no limit

    # =========================================================
    # Publishing
    # =========================================================

    def _publish_setpoint(self, kind, fields):
        now = self._now_us()

        mode = OffboardControlMode()
        mode.timestamp = now
        for name, value in setpoints.control_mode_flags(kind).items():
            setattr(mode, name, value)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        setpoint.position = fields["position"]
        setpoint.velocity = fields["velocity"]
        setpoint.acceleration = fields["acceleration"]
        setpoint.yaw = fields["yaw"]
        setpoint.yawspeed = fields["yawspeed"]

        self._offboard_mode_pub.publish(mode)
        self._trajectory_pub.publish(setpoint)

    def _send_setpoint(self, kind, fields):
        with self._setpoint_lock:
            self._last_setpoint = (kind, fields)
            self._last_user_publish_time = time.time()
            self._publish_setpoint(kind, fields)

    def send_position_setpoint(self, x, y, z, yaw=None, yaw_from_direction=False):
        """
        Fly to an ENU position (metres, relative to PX4's local origin).

        yaw: ENU heading in radians. If None, the locked mission yaw (or current yaw) is held.
        yaw_from_direction: face the direction of travel instead (ignores `yaw`).
        """
        try:
            x = float(x)
            y = float(y)
            z = self._clamp_alt(float(z))

            calculated_yaw = None
            if yaw_from_direction:
                current = self.get_location()
                if current:
                    dx = x - current["x"]
                    dy = y - current["y"]
                    if math.hypot(dx, dy) > 0.3:
                        calculated_yaw = math.atan2(dy, dx)
                    else:
                        calculated_yaw = (
                            self._mission_yaw
                            if self._mission_yaw is not None
                            else self.get_current_yaw()
                        )

            final_yaw = calculated_yaw if calculated_yaw is not None else yaw
            if final_yaw is None:
                final_yaw = (
                    self._mission_yaw
                    if self._mission_yaw is not None
                    else self.get_current_yaw()
                )

            self._send_setpoint(
                "position", setpoints.position_setpoint(x, y, z, final_yaw)
            )
            return True
        except Exception as e:
            print(f"[PX4][ERROR] Failed to publish position setpoint: {str(e)}")
            return False

    def send_velocity_setpoint(self, vx, vy, vz, yaw_rate=0.0):
        """Fly at an ENU velocity (m/s) with an ENU yaw rate (rad/s, counter-clockwise positive)."""
        try:
            self._send_setpoint(
                "velocity", setpoints.velocity_setpoint(vx, vy, vz, yaw_rate)
            )
            return True
        except Exception as e:
            print(f"[PX4][ERROR] Failed to publish velocity setpoint: {str(e)}")
            return False

    def hold_current_position(self):
        loc = self.get_location()
        if not loc:
            print(
                "[PX4][WARN] Cannot hold current position because local position is unavailable"
            )
            return False
        return self.send_position_setpoint(loc["x"], loc["y"], loc["z"])

    # =========================================================
    # Limits and yaw
    # =========================================================

    def set_altitude_limit_ft(self, feet=DEFAULT_MAX_ALT_FT):
        """
        Clamp every position setpoint's altitude in software.

        drone-2026 also tried to set a geofence parameter; parameters are now set in
        QGroundControl (see docs/px4_setup.md), so this is only the software clamp.
        """
        self._max_alt_m = float(feet) * FT_TO_M
        print(f"[PX4] Software altitude limit: {feet} ft ({self._max_alt_m:.2f} m)")
        return True

    def _clamp_alt(self, z):
        if self._max_alt_m is None or z is None:
            return z
        if z > self._max_alt_m:
            print(
                f"[PX4][WARN] Setpoint z={z:.2f}m exceeds limit {self._max_alt_m:.2f}m; clamping"
            )
            return self._max_alt_m
        return z

    def lock_current_yaw(self):
        self._mission_yaw = self.get_current_yaw()
        print(f"[PX4] Mission yaw locked: {math.degrees(self._mission_yaw):.1f}° (ENU)")
        return self._mission_yaw

    # =========================================================
    # Entering OFFBOARD and keeping it alive
    # =========================================================

    def _prime_offboard_stream(self, count=20, dt=0.05):
        """PX4 only accepts OFFBOARD once setpoints are already arriving."""
        if self._stream_running:
            return
        for _ in range(count):
            self.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            time.sleep(dt)

    def start_offboard(self, warmup_count=20, warmup_dt=0.05):
        """Warm up the setpoint stream with zero velocity, then switch to OFFBOARD."""
        if not self.connected:
            print("[PX4] Not connected to PX4, cannot start OFFBOARD")
            return False

        print("[PX4] Warming up OFFBOARD setpoints...")
        for _ in range(warmup_count):
            self.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            time.sleep(warmup_dt)
        return self.change_mode("OFFBOARD")

    def start_offboard_stream_background(self, rate_hz=10):
        """
        Start a thread that republishes the last setpoint so OFFBOARD never times out.

        Start it BEFORE switching to OFFBOARD or waiting for the pilot to arm.
        """
        with self._stream_lock:
            if self._stream_running:
                print("[PX4] Heartbeat stream already running")
                return False
            self._stream_running = True
            self._stream_thread = threading.Thread(
                target=self._heartbeat_worker, args=(rate_hz,), daemon=True
            )
            self._stream_thread.start()
        print("[PX4] ✓ Background heartbeat stream started")
        return True

    def stop_offboard_stream_background(self, timeout=5):
        with self._stream_lock:
            if not self._stream_running:
                return True
            self._stream_running = False

        if self._stream_thread:
            self._stream_thread.join(timeout=timeout)
            if self._stream_thread.is_alive():
                print("[PX4] [WARN] Background thread did not stop within timeout")
                return False

        self._stream_thread = None
        print("[PX4] ✓ Background heartbeat stream stopped")
        return True

    def _heartbeat_worker(self, rate_hz):
        """Republish the last setpoint (or zero velocity) whenever the mission code has gone quiet."""
        dt = 1.0 / rate_hz
        publish_count = 0
        log_interval = int(5 * rate_hz)

        try:
            while self._stream_running:
                with self._setpoint_lock:
                    if time.time() - self._last_user_publish_time > dt:
                        if self._last_setpoint is not None:
                            self._publish_setpoint(*self._last_setpoint)
                        else:
                            self._publish_setpoint(
                                "velocity",
                                setpoints.velocity_setpoint(0.0, 0.0, 0.0, 0.0),
                            )
                        publish_count += 1
                        if publish_count % log_interval == 0:
                            print(
                                f"[PX4] Heartbeat alive - published {publish_count} messages"
                            )
                time.sleep(dt)
        except Exception as e:
            print(f"[PX4] Heartbeat worker error: {str(e)}")
            with self._stream_lock:
                self._stream_running = False

    def wait_for_arm_with_heartbeat(self, timeout=60, heartbeat_rate=10):
        """Wait for the pilot to arm (RC switch or QGC) while streaming zero-velocity setpoints."""
        interval = 1.0 / heartbeat_rate
        start = time.time()
        heartbeat_count = 0
        last_log = -1

        print(
            f"[PX4] Waiting for arm (timeout={timeout}s, heartbeat={heartbeat_rate}Hz)..."
        )
        while (time.time() - start) < timeout:
            self.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            heartbeat_count += 1

            if self.is_armed():
                print(
                    f"[PX4] ✓ Vehicle armed in {time.time() - start:.1f}s ({heartbeat_count} heartbeats)"
                )
                return True

            elapsed = int(time.time() - start)
            if elapsed != last_log:
                last_log = elapsed
                print(
                    f"[PX4] Waiting {int(timeout - elapsed)}s... mode={self.get_mode()}, armed={self.is_armed()}"
                )
            time.sleep(interval)

        print(f"[PX4] ✗ Arm timeout after {timeout}s ({heartbeat_count} heartbeats)")
        return False

    def takeoff(self, altitude, timeout=60):
        """
        Climb `altitude` metres above the current position with an OFFBOARD position setpoint.

        OFFBOARD must already be active with the heartbeat running. Arms first if needed.
        The current yaw is locked so the drone does not rotate on the way up.
        """
        if not self.connected:
            print("[PX4] Not connected to PX4, cannot takeoff")
            return False

        current = self.get_location()
        if not current:
            print("[PX4] Cannot takeoff: local position unavailable")
            return False

        altitude = float(altitude)
        target_alt = self._clamp_alt(current["z"] + altitude)
        yaw = self.lock_current_yaw()
        print(
            f"[PX4] Taking off to {target_alt:.2f}m (current: {current['z']:.2f}m, climbing {altitude:.2f}m)..."
        )

        try:
            if not self.is_armed() and not self.arm_vehicle():
                return False

            if not self.send_position_setpoint(
                current["x"], current["y"], target_alt, yaw=yaw
            ):
                print("[PX4] Failed to send takeoff position setpoint")
                return False

            tolerance = max(0.3, 0.05 * altitude)
            start = time.time()
            while (time.time() - start) < timeout:
                loc = self.get_location()
                if loc and loc["z"] >= target_alt - tolerance:
                    print(f"[PX4] Takeoff complete, reached {loc['z']:.2f}m")
                    return True
                time.sleep(0.5)

            print("[PX4] Takeoff timeout")
            return False
        except Exception as e:
            print(f"[PX4] Takeoff failed: {str(e)}")
            return False
