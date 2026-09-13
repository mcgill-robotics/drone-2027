"""
oa_gazebo.launch.py — bring up the ROS 2 side of the Gazebo obstacle-avoidance stack.

Starts:
  1. MicroXRCEAgent          — PX4 SITL  <->  ROS 2 bridge (replaces MAVROS)
  2. gazebo_lidar_bridge     — Gazebo 2-D lidar (gz.msgs.LaserScan) -> sensor_msgs/LaserScan
  3. oa_node                 — the obstacle-avoidance controller

It does NOT start PX4 SITL + Gazebo itself — that is launched from the
PX4-Autopilot tree (`make px4_sitl gz_x500_lidar_2d`). See docs/sitl.md.

Note: uses the custom gazebo_lidar_bridge (not ros_gz_bridge) to work around a
protobuf version mismatch.

Example:
  ros2 launch drone oa_gazebo.launch.py \\
      target_x:=20.0 target_y:=0.0 target_z:=3.0 \\
      gz_lidar_topic:=/world/default/model/x500_lidar_2d_0/link/lidar_sensor_link/sensor/lidar_2d_v2/scan

Targets are in drone.px4's ENU local frame: x = East, y = North, z = Up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── launch arguments ─────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "target_x",
            default_value="20.0",
            description="Target East in the ENU local frame (m)",
        ),
        DeclareLaunchArgument(
            "target_y",
            default_value="0.0",
            description="Target North in the ENU local frame (m)",
        ),
        DeclareLaunchArgument(
            "target_z", default_value="3.0", description="Target altitude / Up (m)"
        ),
        DeclareLaunchArgument(
            "takeoff_alt", default_value="3.0", description="Takeoff altitude (m)"
        ),
        DeclareLaunchArgument(
            "agent_port",
            default_value="8888",
            description="UDP port PX4 SITL's DDS client uses",
        ),
        DeclareLaunchArgument(
            "ros_lidar_topic",
            default_value="/scan",
            description="ROS 2 LaserScan topic the OA node subscribes to",
        ),
        DeclareLaunchArgument(
            "gz_lidar_topic",
            # Default matches PX4 `gz_x500_lidar_2d` in the `default` world.
            # Run `gz topic -l | grep scan` if your model/world differs.
            default_value="/world/default/model/x500_lidar_2d_0/link/lidar_sensor_link/sensor/lidar_2d_v2/scan",
            description="Gazebo (gz-transport) lidar scan topic to bridge",
        ),
    ]

    # ── 1. Micro XRCE-DDS Agent ──────────────────────────────────────────
    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", LaunchConfiguration("agent_port")],
        output="screen",
    )

    # ── 2. Gazebo lidar -> ROS 2 LaserScan bridge ────────────────────────
    gz_bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "drone",
            "gazebo_lidar_bridge",
            "--gz-topic",
            LaunchConfiguration("gz_lidar_topic"),
            "--ros-topic",
            LaunchConfiguration("ros_lidar_topic"),
        ],
        output="screen",
    )

    # ── 3. OA controller node (the agent above is already running) ───────
    oa_node = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "drone",
            "oa_node",
            "--sitl",
            "--no-agent",
            "--target-x",
            LaunchConfiguration("target_x"),
            "--target-y",
            LaunchConfiguration("target_y"),
            "--target-z",
            LaunchConfiguration("target_z"),
            "--takeoff-alt",
            LaunchConfiguration("takeoff_alt"),
            "--lidar-topic",
            LaunchConfiguration("ros_lidar_topic"),
        ],
        output="screen",
    )

    return LaunchDescription(args + [agent, gz_bridge, oa_node])
