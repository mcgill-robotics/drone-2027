"""
PX4 uXRCE-DDS topic names.

PX4 publishes on /fmu/out/<name> and listens on /fmu/in/<name>. Since PX4 v1.16,
a message whose definition has MESSAGE_VERSION > 0 gets a "_v<N>" suffix, e.g.
VehicleStatus v1 -> /fmu/out/vehicle_status_v1. Reading the version from the
message class keeps these names right when px4_msgs moves to a newer release.
"""


def versioned_name(msg_type, base_name):
    """Append "_v<N>" when the message type declares MESSAGE_VERSION > 0."""
    version = int(getattr(msg_type, "MESSAGE_VERSION", 0))
    return f"{base_name}_v{version}" if version > 0 else base_name


def _prefix(namespace):
    namespace = (namespace or "").strip("/")
    return f"/{namespace}" if namespace else ""


def out_topic(msg_type, base_name, namespace=""):
    """Topic PX4 publishes on (PX4 -> ROS)."""
    return f"{_prefix(namespace)}/fmu/out/{versioned_name(msg_type, base_name)}"


def in_topic(msg_type, base_name, namespace=""):
    """Topic PX4 subscribes to (ROS -> PX4)."""
    return f"{_prefix(namespace)}/fmu/in/{versioned_name(msg_type, base_name)}"
