"""
Flying the drone from code: "fly here" / "fly this fast" orders, and takeoff.

OFFBOARD is the PX4 flight mode where it takes these orders from our computer.
PX4's rule: orders must keep arriving at least ~2 times a second, or it assumes our
computer crashed and leaves OFFBOARD. Most of this file deals with that rule.

Each order ("setpoint") goes out as two messages:
    OffboardControlMode   which kind of order (position or velocity); also the
                          "still alive" signal PX4 watches
    TrajectorySetpoint    the numbers, converted from ENU to NED by setpoints.py

Your code sends an order once. The heartbeat thread (start_offboard_stream_background)
keeps re-sending the last one every 0.1 s, so PX4 stays in OFFBOARD while your code
waits. Replaces the setpoint code and heartbeat thread in drone-2026's px4_setters.py.
"""

import math
import threading
import time

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

from drone.px4 import setpoints
from drone.px4.qos import PX4_QOS
from drone.px4.topics import in_topic

FT_TO_M = 0.3048
DEFAULT_MAX_ALT_FT = 400  # legal height limit for drones


class OffboardMixin:
    """
    The flying part of PX4Interface.

    Not used on its own: PX4Interface combines it with the other mixins, and it
    relies on their methods (get_location(), arm_vehicle(), _now_us(), ...).
    """

    def _init_offboard(self):
        """Set up the two publishers and the heartbeat's state."""
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
        # Stops your code and the heartbeat thread from sending at the same moment
        self._setpoint_lock = threading.Lock()
        # The last order sent, as (kind, fields), so the heartbeat can repeat it
        self._last_setpoint = None
        self._last_user_publish_time = 0.0  # when your code last sent an order

        self._stream_thread = None  # the heartbeat thread
        self._stream_running = False  # set to False to make the heartbeat stop
        self._stream_lock = threading.Lock()

        self._mission_yaw = None  # heading locked at takeoff; later moves keep it
        self._max_alt_m = None  # height ceiling in metres; None = no ceiling

    # =========================================================
    # Sending orders
    # =========================================================

    def _publish_setpoint(self, kind, fields):
        """Send one order: an OffboardControlMode and a TrajectorySetpoint."""
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
        """Send an order from your code, and remember it for the heartbeat to repeat."""
        with self._setpoint_lock:
            self._last_setpoint = (kind, fields)
            self._last_user_publish_time = time.time()
            self._publish_setpoint(kind, fields)

    def send_position_setpoint(self, x, y, z, yaw=None, yaw_from_direction=False):
        """
        Fly to a position: x metres East, y North, z Up from PX4's local origin.

        yaw: which way to face, in radians (0 = East, pi/2 = North). If None, keep
             the heading locked at takeoff (or the current heading).
        yaw_from_direction: face the way you are flying instead (ignores `yaw`).

        Sends the order once and returns; it does not wait for the drone to arrive.
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
        """
        Fly at a speed: vx m/s East, vy North, vz Up.

        yaw_rate: how fast to turn, in rad/s (positive = counter-clockwise).
        """
        try:
            self._send_setpoint(
                "velocity", setpoints.velocity_setpoint(vx, vy, vz, yaw_rate)
            )
            return True
        except Exception as e:
            print(f"[PX4][ERROR] Failed to publish velocity setpoint: {str(e)}")
            return False

    def hold_current_position(self):
        """Stay where you are: send the current position as the target."""
        loc = self.get_location()
        if not loc:
            print(
                "[PX4][WARN] Cannot hold current position because local position is unavailable"
            )
            return False
        return self.send_position_setpoint(loc["x"], loc["y"], loc["z"])

    # =========================================================
    # Height ceiling and heading
    # =========================================================

    def set_altitude_limit_ft(self, feet=DEFAULT_MAX_ALT_FT):
        """
        Set a height ceiling. Any later position order above it is lowered to it.

        This is only a check in our code, not a PX4 geofence; set PX4's own limits
        in QGroundControl (docs/px4_setup.md).
        """
        self._max_alt_m = float(feet) * FT_TO_M
        print(f"[PX4] Software altitude limit: {feet} ft ({self._max_alt_m:.2f} m)")
        return True

    def _clamp_alt(self, z):
        """Lower z to the height ceiling if it is above it."""
        if self._max_alt_m is None or z is None:
            return z
        if z > self._max_alt_m:
            print(
                f"[PX4][WARN] Setpoint z={z:.2f}m exceeds limit {self._max_alt_m:.2f}m; clamping"
            )
            return self._max_alt_m
        return z

    def lock_current_yaw(self):
        """Remember the current heading so later moves keep facing this way."""
        self._mission_yaw = self.get_current_yaw()
        print(f"[PX4] Mission yaw locked: {math.degrees(self._mission_yaw):.1f}° (ENU)")
        return self._mission_yaw

    # =========================================================
    # Entering OFFBOARD and staying in it
    # =========================================================

    def _prime_offboard_stream(self, count=20, dt=0.05):
        """
        Send 20 "stay still" orders, because PX4 refuses OFFBOARD until orders arrive.

        Skipped if the heartbeat thread is already sending orders.
        """
        if self._stream_running:
            return
        for _ in range(count):
            self.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            time.sleep(dt)

    def start_offboard(self, warmup_count=20, warmup_dt=0.05):
        """
        Switch to OFFBOARD: send 20 "stay still" orders first, then ask for the mode.

        Start the heartbeat (start_offboard_stream_background) before this, or PX4
        will leave OFFBOARD again as soon as the orders stop.
        """
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
        Start the heartbeat thread, which re-sends the last order `rate_hz` times a second.

        This keeps PX4 in OFFBOARD while your code is busy waiting. Start it BEFORE
        switching to OFFBOARD or waiting for the pilot to arm.
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
        """Stop the heartbeat thread. PX4 will leave OFFBOARD shortly after."""
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
        """
        The heartbeat thread's loop.

        Every 1/rate_hz seconds: if your code hasn't sent an order since the last
        tick, re-send the last one (or "stay still" if there isn't one yet).
        """
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
        """
        Wait for the pilot to arm (RC switch or QGroundControl). Returns False on timeout.

        Keeps sending "stay still" orders while waiting, so PX4 stays in OFFBOARD.
        """
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
        Climb `altitude` metres straight up, and wait until the drone gets there.

        Needs OFFBOARD active and the heartbeat running. Arms first if needed, and
        locks the current heading so the drone doesn't spin on the way up.
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
