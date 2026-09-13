"""
Unit tests for the ROS-free parts of drone.px4.

These run anywhere (`pytest` from the repo root, and in CI) because frames, topics,
modes, setpoints, actuators, convert and agent do not import rclpy or px4_msgs.
"""

import argparse
import math
from types import SimpleNamespace

import pytest

from drone.px4 import actuators, agent, convert, frames, modes, setpoints
from drone.px4.topics import in_topic, out_topic, versioned_name


def close(a, b, tol=1e-9):
    return math.isclose(a, b, abs_tol=tol)


def angle_close(a, b, tol=1e-9):
    return abs(frames.wrap_pi(a - b)) < tol


# ---------------------------------------------------------------------------
# frames
# ---------------------------------------------------------------------------
class TestFrames:
    def test_ned_to_enu_axes(self):
        # 1 m North, 2 m East, 3 m Down -> 2 m East, 1 m North, 3 m below the origin
        assert frames.ned_to_enu(1.0, 2.0, 3.0) == (2.0, 1.0, -3.0)

    def test_position_round_trip(self):
        enu = (3.0, -4.0, 5.0)
        assert frames.ned_to_enu(*frames.enu_to_ned(*enu)) == enu

    @pytest.mark.parametrize(
        "heading_ned, yaw_enu",
        [
            (0.0, math.pi / 2),  # facing North
            (math.pi / 2, 0.0),  # facing East
            (math.pi, -math.pi / 2),  # facing South
            (-math.pi / 2, math.pi),  # facing West
        ],
    )
    def test_yaw_ned_to_enu(self, heading_ned, yaw_enu):
        assert angle_close(frames.yaw_ned_to_enu(heading_ned), yaw_enu)

    @pytest.mark.parametrize("yaw", [-3.0, -1.0, 0.0, 0.5, 2.9])
    def test_yaw_round_trip(self, yaw):
        assert angle_close(frames.yaw_enu_to_ned(frames.yaw_ned_to_enu(yaw)), yaw)

    def test_yaw_rate_flips_sign(self):
        assert frames.yaw_rate_enu_to_ned(0.3) == -0.3
        assert frames.yaw_rate_ned_to_enu(-0.3) == 0.3

    def test_identity_quaternion(self):
        roll, pitch, yaw = frames.quat_wxyz_to_euler(1.0, 0.0, 0.0, 0.0)
        assert close(roll, 0.0) and close(pitch, 0.0) and close(yaw, 0.0)

    def test_quaternion_yaw_90(self):
        half = math.pi / 4
        _, _, yaw = frames.quat_wxyz_to_euler(math.cos(half), 0.0, 0.0, math.sin(half))
        assert close(yaw, math.pi / 2)

    def test_euler_ned_to_enu(self):
        roll, pitch, yaw = frames.euler_ned_to_enu(0.1, 0.2, 0.0)
        assert close(roll, 0.1) and close(pitch, -0.2) and angle_close(yaw, math.pi / 2)


# ---------------------------------------------------------------------------
# topics
# ---------------------------------------------------------------------------
class _V0:
    MESSAGE_VERSION = 0


class _V1:
    MESSAGE_VERSION = 1


class TestTopics:
    def test_unversioned(self):
        assert (
            out_topic(_V0, "vehicle_local_position")
            == "/fmu/out/vehicle_local_position"
        )

    def test_versioned_suffix(self):
        assert out_topic(_V1, "vehicle_status") == "/fmu/out/vehicle_status_v1"

    def test_no_version_attribute(self):
        assert versioned_name(object, "trajectory_setpoint") == "trajectory_setpoint"

    def test_namespace(self):
        assert (
            in_topic(_V0, "vehicle_command", "/drone1/")
            == "/drone1/fmu/in/vehicle_command"
        )


# ---------------------------------------------------------------------------
# modes
# ---------------------------------------------------------------------------
class _FakeVehicleStatus:
    NAVIGATION_STATE_POSCTL = 2
    NAVIGATION_STATE_AUTO_RTL = 5
    NAVIGATION_STATE_OFFBOARD = 14
    NAVIGATION_STATE_STAB = 15
    NAVIGATION_STATE_MAX = 31


class TestModes:
    def test_offboard_uses_px4_example_values(self):
        assert modes.mode_command("OFFBOARD") == (
            176,
            (1.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def test_aliases_match_full_names(self):
        assert modes.mode_command("hold") == modes.mode_command("AUTO.LOITER")
        assert modes.mode_command("auto_loiter")[1][1:3] == (4.0, 3.0)

    def test_rtl_and_land_use_nav_commands(self):
        assert modes.mode_command("RTL")[0] == 20
        command, params = modes.mode_command("auto.land")
        assert command == 21
        assert all(math.isnan(p) for p in params[3:])

    def test_unknown_mode(self):
        with pytest.raises(ValueError):
            modes.mode_command("GUIDED")  # ArduPilot mode, does not exist in PX4

    def test_nav_state_names(self):
        assert modes.nav_state_names(_FakeVehicleStatus) == {
            2: "POSCTL",
            5: "AUTO.RTL",
            14: "OFFBOARD",
            15: "STABILIZED",
        }

    def test_nav_state_name_round_trips_through_normalize(self):
        for name in modes.nav_state_names(_FakeVehicleStatus).values():
            assert modes.normalize_mode_name(name) == name


# ---------------------------------------------------------------------------
# setpoints
# ---------------------------------------------------------------------------
class TestSetpoints:
    def test_position_setpoint_converts_to_ned(self):
        # 10 m East, 20 m North, 5 m up, facing North
        fields = setpoints.position_setpoint(10.0, 20.0, 5.0, yaw=math.pi / 2)
        assert fields["position"] == [20.0, 10.0, -5.0]
        assert close(fields["yaw"], 0.0)
        assert all(math.isnan(v) for v in fields["velocity"] + fields["acceleration"])
        assert math.isnan(fields["yawspeed"])

    def test_position_setpoint_without_yaw(self):
        assert math.isnan(setpoints.position_setpoint(0, 0, 1)["yaw"])

    def test_velocity_setpoint_converts_to_ned(self):
        # 1 m/s East, climbing 0.5 m/s, turning left (counter-clockwise) at 0.2 rad/s
        fields = setpoints.velocity_setpoint(1.0, 0.0, 0.5, yaw_rate=0.2)
        assert fields["velocity"] == [0.0, 1.0, -0.5]
        assert close(fields["yawspeed"], -0.2)
        assert all(math.isnan(v) for v in fields["position"])
        assert math.isnan(fields["yaw"])

    def test_control_mode_flags(self):
        flags = setpoints.control_mode_flags("velocity")
        assert flags["velocity"] and not flags["position"]
        assert not any(
            flags[k]
            for k in ("acceleration", "attitude", "body_rate", "thrust_and_torque")
        )
        with pytest.raises(ValueError):
            setpoints.control_mode_flags("attitude")


# ---------------------------------------------------------------------------
# actuators
# ---------------------------------------------------------------------------
class TestActuators:
    def test_pwm_mapping(self):
        assert actuators.pwm_to_actuator(1500) == 0.0
        assert close(actuators.pwm_to_actuator(1900), 0.8)
        assert actuators.pwm_to_actuator(1000) == -1.0
        assert actuators.pwm_to_actuator(2500) == 1.0

    def test_params_set_only_one_slot(self):
        params = actuators.actuator_command_params(actuators.PAYLOAD_A, 0.8)
        assert len(params) == 7
        assert close(params[1], 0.8)
        assert all(math.isnan(params[i]) for i in (0, 2, 3, 4, 5))
        assert params[6] == 0.0

    def test_value_clamped(self):
        assert actuators.actuator_command_params(actuators.SPRAY, 5)[0] == 1.0

    @pytest.mark.parametrize("slot", [0, 7, 8])
    def test_slot_out_of_range(self, slot):
        # drone-2026's UI sent "channel 8", which PX4's six actuator sets cannot address
        with pytest.raises(ValueError):
            actuators.actuator_command_params(slot, 1.0)


# ---------------------------------------------------------------------------
# convert
# ---------------------------------------------------------------------------
class TestConvert:
    def test_local_position_to_enu(self):
        msg = SimpleNamespace(xy_valid=True, z_valid=True, x=1.0, y=2.0, z=-3.0)
        assert convert.local_position_enu(msg) == {"x": 2.0, "y": 1.0, "z": 3.0}

    def test_local_position_invalid(self):
        msg = SimpleNamespace(xy_valid=False, z_valid=True, x=1.0, y=2.0, z=-3.0)
        assert convert.local_position_enu(msg) is None

    def test_heading_north_is_enu_pi_over_2(self):
        assert close(convert.heading_enu(SimpleNamespace(heading=0.0)), math.pi / 2)

    def test_heading_nan(self):
        assert convert.heading_enu(SimpleNamespace(heading=float("nan"))) is None

    def test_zero_quaternion_is_invalid(self):
        assert convert.attitude_ned(SimpleNamespace(q=[0.0, 0.0, 0.0, 0.0])) is None

    def test_battery_unknown_remaining(self):
        msg = SimpleNamespace(voltage_v=15.2, current_a=3.0, remaining=-1.0)
        assert convert.battery(msg)["percentage"] is None

    def test_global_position_alt_invalid(self):
        msg = SimpleNamespace(
            lat_lon_valid=True, alt_valid=False, lat=45.5, lon=-73.6, alt=30.0
        )
        assert convert.global_position(msg) == {
            "latitude": 45.5,
            "longitude": -73.6,
            "altitude": None,
        }


# ---------------------------------------------------------------------------
# agent
# ---------------------------------------------------------------------------
def _parse(*argv):
    parser = argparse.ArgumentParser()
    agent.add_link_args(parser)
    return parser.parse_args(list(argv))


class TestAgent:
    def test_sitl(self):
        assert agent.agent_command(_parse("--sitl")) == [
            "MicroXRCEAgent",
            "udp4",
            "-p",
            "8888",
        ]

    def test_udp_default_and_custom_port(self):
        assert agent.agent_command(_parse("--udp"))[-1] == "8888"
        assert agent.agent_command(_parse("--udp", "9999"))[-1] == "9999"

    def test_serial_defaults(self):
        assert agent.agent_command(_parse("--serial")) == [
            "MicroXRCEAgent",
            "serial",
            "--dev",
            "/dev/ttyTHS1",
            "-b",
            "921600",
        ]

    def test_serial_custom(self):
        cmd = agent.agent_command(_parse("--serial", "/dev/ttyUSB0", "--baud", "57600"))
        assert cmd[3] == "/dev/ttyUSB0" and cmd[5] == "57600"

    def test_no_link_chosen(self, monkeypatch):
        monkeypatch.setattr(agent, "HARDWARE_DEFAULT_LINK", None)
        with pytest.raises(ValueError):
            agent.agent_command(_parse())

    def test_link_flags_round_trip(self):
        args = _parse("--udp", "9000")
        child = _parse(*agent.link_flags(args, no_agent=True))
        assert agent.agent_command(child) == agent.agent_command(args)
        assert child.no_agent

    def test_links_are_mutually_exclusive(self):
        with pytest.raises(SystemExit):
            _parse("--sitl", "--serial")
