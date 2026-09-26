"""
One-off commands to PX4: arm, disarm, change mode, land, and payload/gimbal servos.

A command is a VehicleCommand message: a command number (see modes.py) plus up to
7 numeric parameters. PX4 usually answers with a VehicleCommandAck ("ack") saying
whether it accepted. drone-2026 did the same through MAVROS service calls.

Two ways to send:
  - send_command():    send, then wait for PX4's ack. Used for arm, mode changes, land.
  - publish_command(): send and don't wait. Used for servo and gimbal commands,
                       because PX4 never acks those.

While send_command() waits, the ack arrives on the background thread from
interface.py, which wakes the waiting code up.
"""

import threading
import time

from px4_msgs.msg import VehicleCommand, VehicleCommandAck

from drone.px4 import actuators, modes
from drone.px4.qos import PX4_QOS
from drone.px4.topics import in_topic, out_topic

# Which vehicle and which part of it the command is for. 1/1 = PX4 on our drone.
TARGET_SYSTEM = 1
TARGET_COMPONENT = 1

# PX4's ack result numbers, as names for error messages.
ACK_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}


class CommandsMixin:
    """
    The one-off-commands part of PX4Interface.

    Not used on its own: PX4Interface combines it with the other mixins, and it
    relies on PX4Interface's `namespace` and `_now_us()`.
    """

    def _init_commands(self):
        """Set up the command publisher and the ack subscriber."""
        self._command_pub = self.create_publisher(
            VehicleCommand,
            in_topic(VehicleCommand, "vehicle_command", self.namespace),
            PX4_QOS,
        )
        self._ack_sub = self.create_subscription(
            VehicleCommandAck,
            out_topic(VehicleCommandAck, "vehicle_command_ack", self.namespace),
            self._ack_callback,
            PX4_QOS,
        )
        self._ack_lock = threading.Lock()
        # Commands waiting for an ack: command number -> [event to wake the waiter, result]
        self._ack_waiters = {}
        self._small_motor_active = False
        self._check_command_ids()

    def _check_command_ids(self):
        """Warn if the command numbers in modes.py don't match the installed px4_msgs."""
        expected = {
            "VEHICLE_CMD_COMPONENT_ARM_DISARM": modes.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            "VEHICLE_CMD_DO_SET_MODE": modes.VEHICLE_CMD_DO_SET_MODE,
            "VEHICLE_CMD_NAV_LAND": modes.VEHICLE_CMD_NAV_LAND,
            "VEHICLE_CMD_NAV_RETURN_TO_LAUNCH": modes.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
            "VEHICLE_CMD_DO_SET_ACTUATOR": modes.VEHICLE_CMD_DO_SET_ACTUATOR,
            "VEHICLE_CMD_DO_MOUNT_CONTROL": modes.VEHICLE_CMD_DO_MOUNT_CONTROL,
        }
        for name, value in expected.items():
            actual = getattr(VehicleCommand, name, None)
            if actual is None or int(actual) != value:
                print(
                    f"[PX4][WARN] {name} is {actual} in px4_msgs but {value} in drone/px4/modes.py"
                )

    # =========================================================
    # Sending
    # =========================================================

    def _build_command(self, command, params):
        """Fill in a VehicleCommand message. Missing params are padded with 0."""
        values = [float(p) for p in params]
        if len(values) > 7:
            raise ValueError("VehicleCommand has at most 7 params")
        values += [0.0] * (7 - len(values))

        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        (
            msg.param1,
            msg.param2,
            msg.param3,
            msg.param4,
            msg.param5,
            msg.param6,
            msg.param7,
        ) = values
        msg.command = int(command)
        msg.target_system = TARGET_SYSTEM
        msg.target_component = TARGET_COMPONENT
        msg.confirmation = 0
        msg.from_external = True
        return msg

    def publish_command(self, command, params=()):
        """Send a command and return straight away. For commands PX4 never answers."""
        self._command_pub.publish(self._build_command(command, params))

    def send_command(self, command, params=(), timeout=3.0):
        """
        Send a command and wait for PX4's answer.

        Returns PX4's result number: 0 means accepted, anything else is a rejection
        (see ACK_RESULT_NAMES). Returns None if PX4 didn't answer within `timeout`.
        """
        command = int(command)
        waiter = [threading.Event(), None]
        with self._ack_lock:
            self._ack_waiters[command] = waiter
        try:
            self._command_pub.publish(self._build_command(command, params))
            waiter[0].wait(timeout)
            return waiter[1]
        finally:
            with self._ack_lock:
                if self._ack_waiters.get(command) is waiter:
                    del self._ack_waiters[command]

    def _ack_callback(self, msg):
        """Runs when an ack arrives: hand the result to whoever is waiting for it."""
        with self._ack_lock:
            waiter = self._ack_waiters.get(int(msg.command))
        if waiter is None:
            return
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS:
            return  # "still working on it"; the real answer comes later
        waiter[1] = int(msg.result)
        waiter[0].set()

    def _command_ok(self, label, result):
        """True if `result` means accepted; otherwise print why not."""
        if result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return True
        if result is None:
            print(f"[PX4] {label}: no reply from PX4")
        else:
            print(
                f"[PX4] {label} rejected by PX4: {ACK_RESULT_NAMES.get(result, result)}"
            )
        return False

    def _wait_until(self, predicate, timeout):
        """Check predicate() every 0.1 s until it is True (return True) or time runs out."""
        start = time.time()
        while (time.time() - start) < timeout:
            if predicate():
                return True
            time.sleep(0.1)
        return False

    # =========================================================
    # Arming, modes, landing
    # =========================================================

    def arm_vehicle(self, timeout=20):
        """
        Arm the motors from code, and wait until PX4 reports armed.

        On the real drone the pilot arms with the RC switch instead (docs/arming.md);
        this is for the simulator and props-off bench tests.
        """
        if not self.connected:
            print("[PX4] Not connected to PX4, cannot arm")
            return False
        if self.is_armed():
            print("[PX4] Vehicle already armed")
            return True

        print("[PX4] Arming vehicle...")
        result = self.send_command(
            modes.VEHICLE_CMD_COMPONENT_ARM_DISARM, (1.0,), timeout=min(timeout, 5.0)
        )
        if not self._command_ok("Arm", result):
            return False
        if self._wait_until(self.is_armed, timeout):
            print("[PX4] Vehicle armed successfully")
            return True
        print("[PX4] Arm accepted but vehicle_status never reported armed")
        return False

    def disarm_vehicle(self, timeout=20):
        """Disarm the motors, and wait until PX4 reports disarmed."""
        if not self.connected:
            print("[PX4] Not connected to PX4, cannot disarm")
            return False
        if not self.is_armed():
            print("[PX4] Vehicle already disarmed")
            return True

        print("[PX4] Disarming vehicle...")
        result = self.send_command(
            modes.VEHICLE_CMD_COMPONENT_ARM_DISARM, (0.0,), timeout=min(timeout, 5.0)
        )
        if not self._command_ok("Disarm", result):
            return False
        if self._wait_until(lambda: not self.is_armed(), timeout):
            print("[PX4] Vehicle disarmed successfully")
            return True
        print("[PX4] Disarm accepted but vehicle_status still reports armed")
        return False

    def change_mode(self, mode_name, timeout=10):
        """
        Ask PX4 to switch flight mode, e.g. "OFFBOARD", "POSCTL", "HOLD", "RTL", "LAND".

        Returns True if PX4 accepts the request. Call wait_for_mode() afterwards to
        confirm it really switched.

        PX4 refuses OFFBOARD unless "fly here" orders are already arriving, so for
        OFFBOARD this first sends a burst of "stay still" orders (unless the
        heartbeat thread from offboard.py is already sending them).
        """
        if not self.connected:
            print("[PX4] Not connected to PX4, cannot change mode")
            return False
        try:
            command, params = modes.mode_command(mode_name)
        except ValueError as e:
            print(f"[PX4] {e}")
            return False

        if modes.normalize_mode_name(mode_name) == "OFFBOARD":
            self._prime_offboard_stream()

        print(f"[PX4] Changing mode to {mode_name}...")
        result = self.send_command(command, params, timeout=timeout)
        if self._command_ok(f"Mode change to {mode_name}", result):
            print(f"[PX4] Mode change to {mode_name} accepted")
            return True
        return False

    def land(self, timeout=60):
        """
        Switch to PX4's LAND mode and wait until the drone is on the ground.

        PX4 flies the descent and detects touchdown itself.
        """
        if not self.connected:
            print("[PX4] Not connected to PX4, cannot land")
            return False

        print("[PX4] Requesting PX4 landing mode...")
        command, params = modes.mode_command("LAND")
        result = self.send_command(command, params, timeout=5.0)
        if result is None:
            print("[PX4] Landing command reply timed out; monitoring landing state")
        elif result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self._command_ok("Land", result)
            return False
        else:
            print("[PX4] Landing command accepted")

        start = time.time()
        while (time.time() - start) < timeout:
            if self.is_landed():
                print("[PX4] Landing complete, vehicle reports landed")
                return True
            loc = self.get_location()
            if loc and loc["z"] < 0.1:
                print(f"[PX4] Landing complete, reached {loc['z']:.2f}m")
                return True
            time.sleep(0.5)

        print("[PX4] Landing timeout")
        return False

    # =========================================================
    # Payload, spray and gimbal servos (see actuators.py)
    # =========================================================

    def _send_actuator_command(self, actuator_slot, actuator_value):
        """
        Set one servo/pump output (actuator set 1-6) to a value from -1 to 1.

        PX4 never answers this command, so True only means it was sent, not that
        the servo actually moved.
        """
        if not self.connected:
            print("[PX4] Not connected to PX4, cannot send actuator command")
            return False
        try:
            params = actuators.actuator_command_params(actuator_slot, actuator_value)
        except ValueError as e:
            print(f"[PX4] {e}")
            return False
        self.publish_command(modes.VEHICLE_CMD_DO_SET_ACTUATOR, params)
        return True

    def activate_spray(self, actuator_slot=actuators.SPRAY, actuator_value=1.0):
        """Turn the spray pump on."""
        print(
            f"[PX4] Activating spray pump on actuator set {actuator_slot} (value: {actuator_value})"
        )
        return self._send_actuator_command(actuator_slot, actuator_value)

    def deactivate_spray(self, actuator_slot=actuators.SPRAY, actuator_value=0.0):
        """Turn the spray pump off."""
        print(f"[PX4] Deactivating spray pump on actuator set {actuator_slot}")
        return self._send_actuator_command(actuator_slot, actuator_value)

    def release_payload(
        self,
        slots=(actuators.PAYLOAD_A, actuators.PAYLOAD_B),
        release_pwm=1900,
        neutral_pwm=1500,
        pulse_seconds=0.5,
    ):
        """Drop the payload: move the payload servos to release, wait, then back to neutral."""
        release_value = actuators.pwm_to_actuator(release_pwm)
        neutral_value = actuators.pwm_to_actuator(neutral_pwm)
        print(
            f"[PX4] Releasing payload on actuator sets {tuple(slots)} (value: {release_value:.2f})"
        )

        for slot in slots:
            if not self._send_actuator_command(slot, release_value):
                print(f"[PX4] Payload release failed on actuator set {slot}")
                return False
        time.sleep(max(0.0, float(pulse_seconds)))
        for slot in slots:
            if not self._send_actuator_command(slot, neutral_value):
                print(f"[PX4] Payload neutral reset failed on actuator set {slot}")
                return False

        print("[PX4] Payload release sent")
        return True

    def release_small_payload(
        self,
        slot=actuators.SMALL_PAYLOAD,
        release_pwm=1900,
        neutral_pwm=1500,
        pulse_seconds=0.5,
    ):
        """Same as release_payload(), but for the small payload servo."""
        return self.release_payload(
            slots=(slot,),
            release_pwm=release_pwm,
            neutral_pwm=neutral_pwm,
            pulse_seconds=pulse_seconds,
        )

    def start_small_motor(self, slot=actuators.SMALL_PAYLOAD, pwm_value=1900):
        """Turn the small motor on (it shares the small payload output)."""
        print(
            f"[PX4] Starting small motor on actuator set {slot} (PWM-equivalent: {pwm_value})"
        )
        if self._send_actuator_command(slot, actuators.pwm_to_actuator(pwm_value)):
            self._small_motor_active = True
            return True
        return False

    def stop_small_motor(self, slot=actuators.SMALL_PAYLOAD, neutral_pwm=1500):
        """Turn the small motor off."""
        print(f"[PX4] Stopping small motor on actuator set {slot}")
        if self._send_actuator_command(slot, actuators.pwm_to_actuator(neutral_pwm)):
            self._small_motor_active = False
            return True
        return False

    def toggle_small_motor(
        self, slot=actuators.SMALL_PAYLOAD, pwm_on=1900, neutral_pwm=1500
    ):
        """Turn the small motor on if it is off, or off if it is on."""
        if self._small_motor_active:
            return self.stop_small_motor(slot=slot, neutral_pwm=neutral_pwm)
        return self.start_small_motor(slot=slot, pwm_value=pwm_on)

    def set_gimbal(
        self,
        yaw_pwm=1500,
        pitch_pwm=1500,
        yaw_slot=actuators.GIMBAL_YAW,
        pitch_slot=actuators.GIMBAL_PITCH,
    ):
        """
        Point the gimbal by setting its two servos.

        Values are PWM-style: 1500 = centre, 1000 and 2000 = the two ends.
        """
        print(
            f"[PX4] Setting gimbal: yaw_set={yaw_slot} pwm={yaw_pwm}, pitch_set={pitch_slot} pwm={pitch_pwm}"
        )
        yaw_ok = self._send_actuator_command(
            yaw_slot, actuators.pwm_to_actuator(yaw_pwm)
        )
        pitch_ok = self._send_actuator_command(
            pitch_slot, actuators.pwm_to_actuator(pitch_pwm)
        )
        return yaw_ok and pitch_ok

    def center_gimbal(
        self,
        yaw_slot=actuators.GIMBAL_YAW,
        pitch_slot=actuators.GIMBAL_PITCH,
        neutral_pwm=1500,
    ):
        """Point the gimbal straight ahead (both servos centred)."""
        return self.set_gimbal(
            yaw_pwm=neutral_pwm,
            pitch_pwm=neutral_pwm,
            yaw_slot=yaw_slot,
            pitch_slot=pitch_slot,
        )

    def set_gimbal_angles(self, pitch_deg=0.0, roll_deg=0.0, yaw_deg=0.0):
        """
        Point the gimbal by angle, in degrees.

        Only works if the gimbal is set up in PX4 as a gimbal. Our gimbal is wired
        as two plain servos, so use set_gimbal() instead. PX4 never answers this.
        """
        if not self.connected:
            print("[PX4] Not connected to PX4, cannot set gimbal")
            return False
        # param7 = 2 means "point at the angles given here"
        params = (float(pitch_deg), float(roll_deg), float(yaw_deg), 0.0, 0.0, 0.0, 2.0)
        self.publish_command(modes.VEHICLE_CMD_DO_MOUNT_CONTROL, params)
        print(
            f"[PX4] Gimbal angles sent: pitch={pitch_deg:.1f} roll={roll_deg:.1f} yaw={yaw_deg:.1f}"
        )
        return True
