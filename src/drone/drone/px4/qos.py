"""
The QoS profile used for every PX4 topic.

QoS ("Quality of Service") is the set of delivery rules for a ROS topic. Both sides
of a topic must use compatible rules, or no messages get through at all, and ROS only
prints one easy-to-miss warning.

PX4_QOS matches what PX4 expects. Use it for every publisher and subscriber on a PX4
topic, e.g. create_subscription(VehicleStatus, topic, callback, PX4_QOS).

Do NOT pass a plain number like 10 instead: that makes the ROS default (RELIABLE),
which receives nothing from PX4's BEST_EFFORT topics.
"""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

PX4_QOS = QoSProfile(
    # Send without checking delivery; lost messages are not resent. PX4 uses this.
    reliability=ReliabilityPolicy.BEST_EFFORT,
    # On connecting, receive the most recent message right away.
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    # Keep only the newest message; old positions are useless.
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
