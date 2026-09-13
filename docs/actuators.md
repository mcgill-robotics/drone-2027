# Peripheral outputs: spray, payloads, gimbal

drone-2026 drove the payload and gimbal servos with `MAV_CMD_DO_SET_SERVO` (183).
PX4 1.16 does not support that command (commander answers "unsupported"), so those
buttons could not work. Every peripheral now uses `MAV_CMD_DO_SET_ACTUATOR` (187),
sent as a `VehicleCommand` on `/fmu/in/vehicle_command`. It drives PX4's six
"Peripheral via Actuator Set" output functions.

## Mapping

| Actuator Set | Device | drone-2026 output | Constant in `drone/px4/actuators.py` | Physical output (fill in) |
| --- | --- | --- | --- | --- |
| 1 | Spray pump | Actuator Set 1 | `SPRAY` | |
| 2 | Main payload servo A | servo channel 6 | `PAYLOAD_A` | |
| 3 | Main payload servo B | servo channel 7 | `PAYLOAD_B` | |
| 4 | Small payload / small motor | channel 4 | `SMALL_PAYLOAD` | |
| 5 | Gimbal yaw servo | servo channel 8 | `GIMBAL_YAW` | |
| 6 | Gimbal pitch servo | servo channel 9 | `GIMBAL_PITCH` | |

All six sets are used, so there is no spare. If another output is needed, the gimbal
could move to PX4's own gimbal driver (`set_gimbal_angles`), which frees sets 5 and 6.

## QGC configuration

In QGC → Vehicle Setup → Actuators, for each physical output wired to a device above:

1. Set its function to "Peripheral via Actuator Set N" from the table.
2. Set Minimum / Maximum PWM so -1 → 1000 µs and +1 → 2000 µs (0 → 1500 µs), then
   adjust to each servo's real travel.
3. Set Disarmed PWM to the safe position: pump off, payload latched, gimbal centred.

Write the output numbers into the table above.

## Values

The API and UI still use PWM-style numbers. `pwm_to_actuator()` maps 1000..2000 onto
-1..1 (1500 → 0, 1900 → 0.8). Spray sends 1.0 for on and 0.0 for off.

## No confirmation from PX4

PX4 handles `DO_SET_ACTUATOR` outside its commander and sends no acknowledgement. An
API response of `success: true` only means the command was sent. Watch the device.

## Bench test (props off)

1. `ros2 run drone api_server <link>`, then open `ui/index.html`.
2. Hold **Spray Water**: pump runs. Release: pump stops.
3. **Release Payload**: both payload servos pulse and return.
4. **Small Payload**: the small motor toggles.
5. Arrow keys: the gimbal moves.
6. Repeat 2–5 with the vehicle armed (props off) and disarmed. If an output only
   moves when armed, note it here, because the payloads and gimbal are used before arming.

## API

```
POST /spray                 {"action": "activate" | "deactivate", "slot": 1, "value": 1.0}
POST /payload/release       {"slots": [2, 3], "release_pwm": 1900, "neutral_pwm": 1500, "pulse_seconds": 0.5}
POST /payload/small-release {"slot": 4, "release_pwm": 1900, "neutral_pwm": 1500, "pulse_seconds": 0.5}
POST /payload/small         {"action": "start" | "stop" | "toggle", "slot": 4, "pwm_on": 1900, "neutral_pwm": 1500}
POST /gimbal/set            {"yaw_pwm": 1500, "pitch_pwm": 1500, "yaw_slot": 5, "pitch_slot": 6}
```

All fields are optional; the defaults are the values shown.
