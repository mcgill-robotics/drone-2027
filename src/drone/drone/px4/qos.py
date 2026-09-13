"""
The QoS profile used for every PX4 topic.

PX4 publishes /fmu/out topics BEST_EFFORT + TRANSIENT_LOCAL and subscribes to /fmu/in
topics BEST_EFFORT + VOLATILE. A RELIABLE subscriber (what rclpy gives you when you
pass a plain queue depth like 10) is incompatible with a BEST_EFFORT publisher: it
receives nothing, and the only sign is a one-line warning in the log. This profile is
compatible in both directions.
"""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
