"""
agent.launch.py — start MicroXRCEAgent, the bridge between PX4 and ROS 2.

Every drone tool can also start the agent itself (see drone.px4.agent); use this
launch file when you want the agent running on its own, e.g. in a separate terminal
while running several tools with --no-agent.

Examples:
    ros2 launch drone agent.launch.py                                   # UDP 8888: SITL or Ethernet
    ros2 launch drone agent.launch.py transport:=serial device:=/dev/ttyTHS1 baud:=921600
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "transport",
                default_value="udp",
                choices=["udp", "serial"],
                description="udp for the simulator or an Ethernet link, serial for a UART link",
            ),
            DeclareLaunchArgument(
                "port", default_value="8888", description="UDP port (transport:=udp)"
            ),
            DeclareLaunchArgument(
                "device",
                default_value="/dev/ttyTHS1",
                description="Serial device (transport:=serial)",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="921600",
                description="Serial baud rate (transport:=serial)",
            ),
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", LaunchConfiguration("port")],
                output="screen",
                condition=LaunchConfigurationEquals("transport", "udp"),
            ),
            ExecuteProcess(
                cmd=[
                    "MicroXRCEAgent",
                    "serial",
                    "--dev",
                    LaunchConfiguration("device"),
                    "-b",
                    LaunchConfiguration("baud"),
                ],
                output="screen",
                condition=LaunchConfigurationEquals("transport", "serial"),
            ),
        ]
    )
