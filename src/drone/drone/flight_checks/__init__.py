"""
The 9-step flight verification ladder for drone-2027.

Execute checks in numerical order from Step 1 to Step 9:
  1. check_link:          Verify MicroXRCEAgent & PX4 FMU bridge connectivity
  2. check_telemetry:     Validate sensor health, GPS fix, EKF state, and battery
  3. check_setpoints:     Dry-run ENU <-> NED setpoint math without moving motors
  4. check_offboard:      Validate 10 Hz heartbeat handshake and OFFBOARD mode switch
  5. check_arm:           Test motor arming & disarming (PROPS OFF on bench)
  6. check_hover:         First flight: takeoff to altitude, hover, and land
  7. check_goto_gps:      Navigate to a single GPS coordinate and hold
  8. check_gps_movement:  Fly sequential GPS waypoints facing direction of travel
  9. check_lap:           Complete a multi-waypoint perimeter lap and RTL

See README.md in this directory for detailed commands, flags, and troubleshooting.
"""
