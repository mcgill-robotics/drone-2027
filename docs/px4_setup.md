# PX4 setup for the uXRCE-DDS bridge

drone-2027 talks to PX4 through the Micro XRCE-DDS Agent, not MAVROS. Everything
below is set once in QGroundControl (QGC). The code can no longer read or write PX4
parameters, so this checklist replaces what drone-2026 did from code.

Values were checked against PX4 **v1.16**. If the flight controller runs a newer
release, repeat step 0 for that version.

## 0. Confirm what the drone actually has (do this first)

Two facts are not confirmed yet:

1. **Exact firmware version.** QGC → Vehicle Setup → Summary, or `ver all` in the
   MAVLink console.
   `src/px4_msgs` is pinned to `release/1.16`. If the drone runs another release,
   point the submodule at it and rebuild:

   ```sh
   git -C src/px4_msgs fetch --depth 1 origin release/1.17
   git -C src/px4_msgs checkout FETCH_HEAD
   # and change `branch =` in .gitmodules to match
   colcon build --symlink-install
   ```

   Topic names are built from each message's `MESSAGE_VERSION`, so the code should
   not need changes. The command numbers in `src/drone/drone/px4/modes.py` are checked
   against px4_msgs at startup and a warning is printed if they differ.

2. **How the Jetson is wired to the flight controller.** drone-2026 was
   inconsistent: `api_server.py` used Ethernet (`192.168.144.11`) while the test
   scripts used serial (`/dev/ttyTHS1` at 921600 baud). Check the cable, and in QGC
   look at `MAV_0_CONFIG`, `MAV_1_CONFIG`, `MAV_2_CONFIG` to see which port currently
   carries MAVLink to the Jetson. Then set `HARDWARE_DEFAULT_LINK` in
   `src/drone/drone/px4/agent.py` to `"serial"` or `"udp"` so tools work without a
   link flag.

## 1. Record the current values first (rollback)

MAVROS and the DDS bridge cannot share one port. Switching the Jetson port to the
bridge breaks the drone-2026 stack until it is switched back. Before changing
anything, write the current values of every parameter you are about to touch into
the rollback table at the bottom.

Do **not** change the port QGC uses (telemetry radio).

## 2. Bridge link

### Serial cable

| Parameter | Value | Why |
| --- | --- | --- |
| `MAV_x_CONFIG` for the Jetson's port | Disabled | Free the port from MAVLink |
| `UXRCE_DDS_CFG` | The same port, e.g. `TELEM 2` | Run the DDS client there |
| `SER_TEL2_BAUD` (the baud parameter of that port) | 921600 | Must match `--baud` |

On the Jetson the user running the tools must be able to open the device
(e.g. member of the `dialout` group). Test: `ros2 run drone check_link --serial /dev/ttyTHS1`.

### Ethernet

| Parameter | Value | Why |
| --- | --- | --- |
| `MAV_x_CONFIG` that used Ethernet for the Jetson | Disabled, if nothing else needs it | Avoid two stacks on one link |
| `UXRCE_DDS_CFG` | `Ethernet` | Run the DDS client on Ethernet |
| `UXRCE_DDS_AG_IP` | Jetson's IP as a 32-bit integer (below) | Where the agent listens |
| `UXRCE_DDS_PRT` | 8888 | Must match `--udp` |

`UXRCE_DDS_AG_IP` stores an IPv4 address as a single integer (the default
`2130706433` is `127.0.0.1`). Convert the Jetson's address with:

```sh
python3 -c "import ipaddress,sys; v=int(ipaddress.IPv4Address(sys.argv[1])); print(v, v - 2**32)" 192.168.144.20
```

It prints the plain value and the signed 32-bit form; QGC stores the parameter as a
signed integer, so use the second (negative) number if it rejects the first.
Test: `ros2 run drone check_link --udp 8888`.

### Both links

| Parameter | Value | Why |
| --- | --- | --- |
| `UXRCE_DDS_DOM_ID` | 0 | Must equal `ROS_DOMAIN_ID` on the Jetson (0 if unset) |
| `UXRCE_DDS_SYNCT` | Enabled (default) | Time sync between PX4 and the Jetson |

Reboot the flight controller after changing these.

## 3. Offboard and arming safety

From `docs/arming.md` (unchanged by the port):

| Parameter | Value |
| --- | --- |
| `COM_OF_LOSS_T` | 1.0 |
| `COM_RCL_EXCEPT` | 4 |
| `RC_MAP_ARM_SW`, `COM_ARM_SWISBTN`, `RC_MAP_KILL_SW`, `COM_RC_ARM_HYST` | see `docs/arming.md` |

## 4. Motion limits

drone-2026 set these from code before every flight. Set them once here; the flight
checks and Mission 1 print them as a reminder but cannot verify them.

| Parameter | Value |
| --- | --- |
| `MPC_XY_VEL_MAX` | 2.0 |
| `MPC_XY_CRUISE` | 1.0 |
| `MPC_VEL_MANUAL` | 2.0 |
| `MPC_LAND_SPEED` | 0.4 |
| `MPC_LAND_CRWL` | 0.2 |

## 5. Peripheral outputs

Spray pump, payload servos, small payload and gimbal: see `docs/actuators.md`.

## 6. Verify on the Jetson

```sh
ros2 run drone check_link <link>
ros2 run drone check_telemetry <link>
```

## Rollback record

| Parameter | Value before the DDS bridge | Value now |
| --- | --- | --- |
| | | |
