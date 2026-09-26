"""
Everything our code needs to talk to PX4, the autopilot on the flight controller.

PX4 and ROS 2 are connected by MicroXRCEAgent, a separate program (see agent.py).
Our side is one ROS 2 node, PX4Interface, that reads PX4's data and sends it orders.

Modules in this package:
  - interface:            PX4Interface and init_px4() / shutdown_px4(). Start here.
  - telemetry:            reads data from PX4 (position, battery, GPS, mode, ...)
  - commands:             sends one-off commands (arm, change mode, land, servos)
  - offboard:             sends "fly here" / "fly this fast" orders and keeps them going
  - agent:                starts and stops MicroXRCEAgent; the --sitl/--udp/--serial flags
  - convert_ned_enu:      turns PX4 messages into simple Python dicts, in ENU
  - ned_enu_math_convert: the maths for switching between PX4's axes (NED) and ours (ENU)
  - setpoints:            fills in the numbers for "fly here" / "fly this fast" orders
  - modes:                translates mode names ("OFFBOARD", "LAND") to PX4's numbers
  - actuators:            servo / pump output numbers for the payload, spray and gimbal
  - qos:                  delivery rules every PX4 topic must use
  - topics:               builds PX4 topic names like /fmu/out/vehicle_status_v1

Import what you need directly, e.g. `from drone.px4.interface import init_px4`.

This file deliberately imports nothing. Most modules above are plain Python with no
ROS 2 imports, so the unit tests can run on any laptop; importing interface here
would pull in ROS 2 and break that.
"""
