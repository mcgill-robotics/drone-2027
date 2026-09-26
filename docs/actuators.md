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

The `PX4Interface` methods take PWM-style numbers. `pwm_to_actuator()` maps 1000..2000
onto -1..1 (1500 → 0, 1900 → 0.8). Spray sends 1.0 for on and 0.0 for off.

## No confirmation from PX4

PX4 handles `DO_SET_ACTUATOR` outside its commander and sends no acknowledgement. A
method returning `True` only means the command was sent. Watch the device.

## Bench test (props off)

Start the agent in one terminal (`ros2 launch drone agent.launch.py transport:=serial`,
or leave `transport` at its default `udp` for Ethernet), then drive the outputs from
`python3` in another (after `source install/setup.bash`):

```python
from drone.px4.interface import init_px4, shutdown_px4
px4 = init_px4()

px4.activate_spray()      # pump runs
px4.deactivate_spray()    # pump stops
px4.release_payload()     # both payload servos pulse and return
px4.toggle_small_motor()  # small motor on; call again to stop
px4.set_gimbal(yaw_pwm=1700, pitch_pwm=1300)  # gimbal moves
px4.center_gimbal()

shutdown_px4()
```

Repeat with the vehicle armed (props off) and disarmed. If an output only moves when
armed, note it here, because the payloads and gimbal are used before arming.

## Methods

All in `drone/px4/commands.py`; every argument is optional and defaults to the value shown.

```
activate_spray(actuator_slot=1, actuator_value=1.0)
deactivate_spray(actuator_slot=1, actuator_value=0.0)
release_payload(slots=(2, 3), release_pwm=1900, neutral_pwm=1500, pulse_seconds=0.5)
release_small_payload(slot=4, release_pwm=1900, neutral_pwm=1500, pulse_seconds=0.5)
start_small_motor(slot=4, pwm_value=1900) / stop_small_motor(slot=4, neutral_pwm=1500)
toggle_small_motor(slot=4, pwm_on=1900, neutral_pwm=1500)
set_gimbal(yaw_pwm=1500, pitch_pwm=1500, yaw_slot=5, pitch_slot=6)
center_gimbal(yaw_slot=5, pitch_slot=6, neutral_pwm=1500)
```
