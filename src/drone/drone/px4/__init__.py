"""
PX4 interface over the uXRCE-DDS bridge.

Import the pieces you need directly, e.g. `from drone.px4.interface import init_px4`.
This file deliberately imports nothing: frames, topics, modes, setpoints, actuators,
convert and agent are plain Python and must stay importable without ROS 2 (for unit
tests and CI).
"""
