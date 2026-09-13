"""
The public PX4 interface: one ROS 2 node for every PX4 topic, plus the thread that spins it.

Replaces drone-2026's px4_interface.py. Typical use:

    from drone.px4.interface import init_px4, shutdown_px4

    px4 = init_px4()          # starts ROS 2, the node and its executor thread; waits for PX4
    if px4.connected:
        print(px4.get_location())
    shutdown_px4()

The executor thread started here is the only thing that spins the node. Callers
must never call rclpy.spin() / spin_once() on it; getters always return the latest
data that thread delivered. (drone-2026 spun from several threads at once, which
crashes rclpy.)

The Micro XRCE-DDS Agent must be running first; see drone.px4.agent.
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
    Everything the rest of the code needs from PX4, in ENU.

    - TelemetryMixin: subscriptions + getters
    - CommandsMixin:  arm, modes, land, peripheral outputs
    - OffboardMixin:  setpoints + OFFBOARD heartbeat
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
        """PX4 timestamps are microseconds."""
        return int(self.get_clock().now().nanoseconds / 1000)

    def disconnect(self):
        shutdown_px4()


# Global instance, one per process
_autopilot = None
_executor = None
_spin_thread = None


def _spin():
    try:
        _executor.spin()
    except Exception as e:  # rclpy raises when shut down from another thread
        print(f"[PX4] Executor stopped: {type(e).__name__}")


def init_px4(node_name="px4_interface", namespace="", connect_timeout=30.0):
    """Create the global PX4Interface, start spinning it, and wait up to connect_timeout for PX4."""
    global _autopilot, _executor, _spin_thread

    if _autopilot is not None:
        return _autopilot

    if not rclpy.ok():
        rclpy.init()

    _autopilot = PX4Interface(node_name=node_name, namespace=namespace)
    _executor = SingleThreadedExecutor()
    _executor.add_node(_autopilot)
    _spin_thread = threading.Thread(target=_spin, name="px4-executor", daemon=True)
    _spin_thread.start()

    _autopilot.wait_for_connection(timeout=connect_timeout)
    return _autopilot


def get_px4():
    """The global PX4Interface, or None before init_px4()."""
    return _autopilot


def shutdown_px4():
    """Stop the heartbeat, the executor thread and ROS 2. Safe to call more than once."""
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
