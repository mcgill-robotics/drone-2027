"""
Builds the names of PX4's ROS 2 topics.

PX4 sends data on /fmu/out/<name> and listens for orders on /fmu/in/<name>.
Since PX4 1.16, some messages have a version number added to the name, e.g.
/fmu/out/vehicle_status_v1. These functions read that version from the message
type itself, so the names stay right when px4_msgs is updated.

A wrong topic name gives no error, just no data, so always build names here.
"""


def versioned_name(msg_type, base_name):
    """Add "_v<N>" to the name if the message type has a version, e.g. "vehicle_status_v1"."""
    version = int(getattr(msg_type, "MESSAGE_VERSION", 0))
    return f"{base_name}_v{version}" if version > 0 else base_name


def _prefix(namespace):
    """ "drone1" -> "/drone1"; "" -> "". Only needed when running several drones."""
    namespace = (namespace or "").strip("/")
    return f"/{namespace}" if namespace else ""


def out_topic(msg_type, base_name, namespace=""):
    """Name of a topic PX4 sends data on, e.g. /fmu/out/vehicle_status_v1."""
    return f"{_prefix(namespace)}/fmu/out/{versioned_name(msg_type, base_name)}"


def in_topic(msg_type, base_name, namespace=""):
    """Name of a topic PX4 listens for orders on, e.g. /fmu/in/vehicle_command."""
    return f"{_prefix(namespace)}/fmu/in/{versioned_name(msg_type, base_name)}"
