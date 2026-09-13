"""
Peripheral outputs driven with MAV_CMD_DO_SET_ACTUATOR (VehicleCommand 187).

PX4 offers six "Peripheral via Actuator Set" output functions. In QGroundControl
(Actuators tab) each physical output is assigned one of them; the mapping this
code expects is documented in docs/actuators.md. Values run from -1 to 1 and PX4
maps them onto the output's configured min/max PWM.

drone-2026 drove the payload and gimbal servos with MAV_CMD_DO_SET_SERVO (183),
which PX4 1.16 does not support, so every peripheral now uses an actuator set.

Pure Python: no ROS imports.
"""

import math

SPRAY = 1
PAYLOAD_A = 2
PAYLOAD_B = 3
SMALL_PAYLOAD = 4
GIMBAL_YAW = 5
GIMBAL_PITCH = 6
NUM_SETS = 6

PWM_CENTER = 1500.0
PWM_HALF_RANGE = 500.0


def clamp_unit(value):
    return max(-1.0, min(1.0, float(value)))


def pwm_to_actuator(pwm, center=PWM_CENTER, half_range=PWM_HALF_RANGE):
    """Map a PWM-style number (1000..2000, 1500 = neutral) onto an actuator value (-1..1)."""
    return clamp_unit((float(pwm) - center) / half_range)


def actuator_command_params(slot, value):
    """
    param1..param7 for DO_SET_ACTUATOR that set one actuator set and leave the others alone.

    `slot` is 1-based (Actuator Set 1..6). Untouched sets are NaN, which PX4 reads
    as "no change". param7 selects the first group of six sets.
    """
    index = int(slot) - 1
    if not 0 <= index < NUM_SETS:
        raise ValueError(f"Actuator set {slot} out of range 1-{NUM_SETS}")
    params = [math.nan] * NUM_SETS
    params[index] = clamp_unit(value)
    return params + [0.0]
