"""
PX4 interface over the uXRCE-DDS bridge.

Modules in this package:
  - interface:   Public PX4Interface node and background spin executor thread
  - telemetry:   Subscribes to /fmu/out topics (GPS, battery, local pos, EKF)
  - commands:    Publishes /fmu/in/vehicle_command and waits for command_ack
  - offboard:    10 Hz background heartbeat thread and setpoint streaming
  - frames:      Coordinate conversion (Aviation NED <-> Robotics ENU)
  - setpoints:   TrajectorySetpoint and OffboardControlMode packet builders
  - modes:       PX4 flight mode command parameters and state translation
  - qos:         PX4_QOS delivery contract (Best Effort + Transient Local)
  - topics:      Dynamic topic versioning resolver (e.g. _v1 suffix in 1.17)
  - agent:       MicroXRCEAgent process manager and CLI argument parsing
  - actuators:   Maps Actuator Sets 1-6 via MAV_CMD_DO_SET_ACTUATOR (187)
  - convert:     Decodes raw ROS message structs into clean Python dicts

Import the pieces you need directly, e.g. `from drone.px4.interface import init_px4`.
This file deliberately imports nothing: frames, topics, modes, setpoints, actuators,
convert and agent are plain Python and must stay importable without ROS 2 (for unit
tests and CI).
"""
