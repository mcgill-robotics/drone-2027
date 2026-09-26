"""
The main entry point for talking to PX4. Replaces drone-2026's px4_interface.py.

    from drone.px4.interface import init_px4, shutdown_px4

    px4 = init_px4()          # connect to PX4 (waits until it answers)
    print(px4.get_location())
    shutdown_px4()            # clean up when done

init_px4() creates three things, kept in the variables below:
  - _autopilot:   the PX4Interface object you call methods on
  - _executor:    delivers incoming PX4 messages to _autopilot
  - _spin_thread: a background thread that keeps _executor running

Incoming messages are handled in the background, so your script just calls getters
like get_location() and always gets the latest data. Never call rclpy.spin() or
spin_once() yourself: two threads spinning the same node crashes rclpy.

MicroXRCEAgent must already be running; see agent.py.
"""

import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from drone.px4.commands import CommandsMixin
from drone.px4.offboard import OffboardMixin
from drone.px4.telemetry import TelemetryMixin


class PX4Interface(OffboardMixin, CommandsMixin, TelemetryMixin, Node):
    """
    One ROS 2 node that does everything the rest of the code needs from PX4.

    All positions and angles it takes and returns are in ENU (x East, y North, z Up).
    The methods are split across three files and combined here:
      - TelemetryMixin (telemetry.py): reading data, e.g. get_location(), is_armed()
      - CommandsMixin (commands.py):   one-off commands, e.g. arm_vehicle(), land()
      - OffboardMixin (offboard.py):   flying, e.g. send_position_setpoint(), takeoff()
    """

    def __init__(self, node_name="px4_interface", namespace=""):
        Node.__init__(self, node_name)
        self.namespace = namespace
        self._init_telemetry()
        self._init_commands()
        self._init_offboard()
        print(
            f"[PX4] Initialized uXRCE-DDS interface (namespace: '{namespace or '/'}')"
        )

    def _now_us(self):
        """Current ROS time in microseconds, the unit PX4 expects in every message's timestamp."""
        return int(self.get_clock().now().nanoseconds / 1000)

    def disconnect(self):
        """Same as shutdown_px4()."""
        shutdown_px4()


# Global instance, one per process. All three are set by init_px4() and cleared by shutdown_px4().
_autopilot = None  # the PX4Interface node: getters, commands, setpoints
_executor = None  # delivers incoming ROS messages to _autopilot's callbacks
_spin_thread = None  # background thread that runs _executor.spin()


def _spin():
    """Body of _spin_thread: handle incoming messages until the executor is shut down."""
    try:
        _executor.spin()  # blocks forever, which is why it gets its own thread
    except Exception as e:  # rclpy raises when shut down from another thread
        print(f"[PX4] Executor stopped: {type(e).__name__}")


def init_px4(node_name="px4_interface", namespace="", connect_timeout=30.0):
    """Create the global PX4Interface, start spinning it, and wait up to connect_timeout for PX4."""
    global _autopilot, _executor, _spin_thread

    # Already initialised: hand back the same instance rather than making a second node
    if _autopilot is not None:
        return _autopilot

    # Start ROS 2 in this process unless something else already did
    if not rclpy.ok():
        rclpy.init()

    # Create the node, then an executor that delivers its incoming messages
    _autopilot = PX4Interface(node_name=node_name, namespace=namespace)
    _executor = SingleThreadedExecutor()
    _executor.add_node(_autopilot)

    # Run the executor in the background so the caller's script keeps going.
    # daemon=True: the thread will not keep the program alive on exit.
    _spin_thread = threading.Thread(target=_spin, name="px4-executor", daemon=True)
    _spin_thread.start()

    # Block until PX4 telemetry arrives through the agent, or give up after connect_timeout
    _autopilot.wait_for_connection(timeout=connect_timeout)
    return _autopilot


def get_px4():
    """The PX4Interface that init_px4() created, or None if it has not been called yet."""
    return _autopilot


def shutdown_px4():
    """
    Undo init_px4(): stop the heartbeat, the background thread and ROS 2.

    Each step is wrapped in try/except so one failure does not skip the rest.
    Safe to call more than once.
    """
    global _autopilot, _executor, _spin_thread

    if _autopilot is None:
        return

    try:
        _autopilot.stop_offboard_stream_background()
    except Exception:
        pass
    try:
        _executor.shutdown()
    except Exception:
        pass
    try:
        _autopilot.destroy_node()
    except Exception:
        pass
    rclpy.try_shutdown()
    if _spin_thread is not None:
        _spin_thread.join(timeout=2.0)

    _autopilot = None
    _executor = None
    _spin_thread = None
    print("[PX4] Interface shut down")
