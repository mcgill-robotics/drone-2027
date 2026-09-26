"""
Nine test scripts that check the drone step by step, from "can we talk to PX4?"
up to a full lap. Run them in order; each one assumes the ones before it passed.

  1. check_link:          the agent starts and PX4 data reaches ROS 2
  2. check_telemetry:     position, GPS, battery etc. read correctly (nothing moves)
  3. check_setpoints:     "fly here" orders are sent with the right numbers (not armed)
  4. check_offboard:      PX4 switches to OFFBOARD and stays there (not armed)
  5. check_arm:           the motors arm and disarm (PROPS OFF on the bench)
  6. check_hover:         first flight: take off, hover, land
  7. check_goto_gps:      fly to one GPS point, then land
  8. check_gps_movement:  fly to a GPS point facing the way it's flying
  9. check_lap:           fly a loop of GPS waypoints, then return to launch

See README.md in this directory for the commands, flags and troubleshooting.
"""
