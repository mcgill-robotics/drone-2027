"""
Numbers for controlling the extra servos and pump (payload, spray, gimbal).

PX4 has six spare outputs for this, called "actuator sets" 1-6. In QGroundControl
(Actuators tab) each physical output pin is assigned one of them; which pin is
which is in docs/actuators.md. We set an output to a value from -1 to 1, and PX4
turns that into the pin's PWM signal.

drone-2026 used a different command (DO_SET_SERVO) that PX4 1.16+ doesn't support,
so everything now goes through actuator sets (command 187, DO_SET_ACTUATOR).

No ROS imports, so the unit tests can run it anywhere.
"""

import math

# Which actuator set drives what (must match QGroundControl; see docs/actuators.md)
SPRAY = 1
PAYLOAD_A = 2
PAYLOAD_B = 3
SMALL_PAYLOAD = 4
GIMBAL_YAW = 5
GIMBAL_PITCH = 6
NUM_SETS = 6

# Servo PWM runs 1000-2000 microseconds, with 1500 as the centre
PWM_CENTER = 1500.0
PWM_HALF_RANGE = 500.0


def clamp_unit(value):
    """Limit a value to -1..1."""
    return max(-1.0, min(1.0, float(value)))


def pwm_to_actuator(pwm, center=PWM_CENTER, half_range=PWM_HALF_RANGE):
    """PWM-style number -> actuator value, e.g. 1000 -> -1, 1500 -> 0, 2000 -> 1."""
    return clamp_unit((float(pwm) - center) / half_range)


def actuator_command_params(slot, value):
    """
    The 7 command parameters that set actuator set `slot` (1-6) to `value`.

    The other five sets are sent as NaN, which PX4 reads as "leave unchanged".
    The 7th parameter picks the group of sets; 0 = sets 1-6.
    """
    index = int(slot) - 1
    if not 0 <= index < NUM_SETS:
        raise ValueError(f"Actuator set {slot} out of range 1-{NUM_SETS}")
    params = [math.nan] * NUM_SETS
    params[index] = clamp_unit(value)
    return params + [0.0]
