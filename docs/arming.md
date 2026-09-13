# Arming with a dedicated RC switch

This is the flow we use for hardware tests and race day: the companion
computer handles mode (OFFBOARD), the pilot holds physical arm/kill
authority via dedicated RC switches.

## Required PX4 parameters

Set these via QGroundControl ("Parameters" → search) or MAVLink shell
(`param set ...`). Values are starting points — adjust channel numbers to
your transmitter's wiring.

| Parameter            | Value            | Why                                      |
| -------------------- | ---------------- | ---------------------------------------- |
| `RC_MAP_ARM_SW`      | channel #        | RC channel mapped to your arm switch     |
| `COM_ARM_SWISBTN`    | 0 (toggle) / 1 (momentary) | Match your switch type         |
| `RC_MAP_KILL_SW`     | channel #        | Kill switch — cuts motors instantly      |
| `COM_RC_ARM_HYST`    | 1000 (default)   | Hold-time before arm registers (ms)      |
| `COM_RCL_EXCEPT`     | 4                | Allow OFFBOARD even if RC link lost      |
| `COM_OF_LOSS_T`      | 1.0              | Setpoint-loss timeout (s) — don't lower  |

After setting, **reboot the FCU** and verify in QGC that the switch
appears in the "Radio" tab and toggles `Armed` state.

## Pre-flight verification

Before props on, with the vehicle on a bench:

In the commands below, `<link>` is `--serial` or `--udp`, whichever matches
how the Jetson is wired to the flight controller (see `docs/px4_setup.md`).

1. Power the FCU. Wait for GPS 3D fix (≥6 sats, EKF green).
2. Start the bridge and confirm PX4 is publishing:
   `ros2 run drone check_link <link>`.
3. Run `ros2 run drone check_offboard <link>`. It starts the heartbeat
   stream **before** the mode change, switches to OFFBOARD, and confirms
   PX4 reports OFFBOARD as active.
4. Watch QGC: it should show OFFBOARD active and "Ready to Arm".
5. Run `ros2 run drone check_arm <link>` and flip the arm switch when asked.
   Motors arm.
6. Flip the kill switch. Motors disarm instantly.
7. Check every peripheral output moves (`docs/actuators.md`).

If any step fails, **do not put props on**. The failure modes with props
attached are dangerous.

## Race-day arming flow

```
companion boots → MicroXRCEAgent up → PX4 connected → heartbeat stream → OFFBOARD → pilot arm switch → mission runs → pilot disarm or kill
```

The companion never calls `arm_vehicle()` in this flow. The pilot is the
arming authority. The flight checks and mission runners only arm from code
with `--api` / `--api-arm`, which they refuse unless `--sitl` is also given.
The one exception is `check_arm --api`, meant for props-off bench tests.

## Common gotchas

- **OFFBOARD drops before arm**: `OffboardControlMode` stopped arriving for
  longer than `COM_OF_LOSS_T`. Make sure `start_offboard_stream_background()`
  is running before the arm wait, not just inside it.
- **Switch flipped but nothing happens**: `RC_MAP_ARM_SW` not set, or set
  to the wrong channel. Check the "Radio" tab in QGC to confirm the
  channel moves when you flip the switch.
- **Arms then immediately disarms**: pre-arm check failure that only
  triggers after arming briefly. Usually low battery or EKF reject.
  Read the messages panel in QGC. (MAVROS used to republish these on
  `/mavros/statustext`; the DDS bridge does not carry them.)
  `ros2 topic echo /fmu/out/failsafe_flags` also shows which check fails.
- **Won't arm while in OFFBOARD**: `COM_ARM_SWISBTN` mismatch with switch
  type (toggle vs momentary), or `RC_MAP_ARM_SW` not pointing at a free
  channel.
- **No `/fmu` topics at all**: the agent is not running, the link flag is
  wrong, or `UXRCE_DDS_CFG` does not point at the port the Jetson is wired
  to. See `docs/px4_setup.md`.
