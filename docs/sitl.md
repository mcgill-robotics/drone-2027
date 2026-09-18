# Simulator: PX4 SITL + Gazebo

Run on Ubuntu 22.04 with ROS 2 Humble (a lab PC, the Jetson, or `docker/Dockerfile`).
macOS is not a practical host for PX4 SITL with Gazebo.

## One-time setup

1. PX4-Autopilot at the same release as `src/px4_msgs` (currently 1.17):

   ```sh
   git clone --recursive -b release/1.17 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
   bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
   ```

2. Micro-XRCE-DDS-Agent and this workspace: see the README.

SITL starts PX4's DDS client automatically and connects to UDP port 8888 on
localhost, which is what `--sitl` uses. Nothing needs to be configured in PX4.

## Flight checks

Terminal 1:

```sh
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Terminal 2:

```sh
cd drone-2027 && source install/setup.bash
ros2 run drone check_link --sitl
ros2 run drone check_telemetry --sitl --duration 10
ros2 run drone check_offboard --sitl
ros2 run drone check_arm --sitl --api
ros2 run drone check_hover --sitl --api
ros2 run drone check_goto_gps --sitl --api --lat 47.39785 --lon 8.54573 --alt 10
```

Each tool starts the agent itself. To run two tools at the same time, start the agent
once (`ros2 launch drone agent.launch.py`) and give both `--no-agent`.

`check_lap` and Mission 1 use Montreal coordinates. SITL spawns in Zurich by default,
so move the simulated home there first:

```sh
PX4_HOME_LAT=45.5065 PX4_HOME_LON=-73.5803 PX4_HOME_ALT=30 make px4_sitl gz_x500
ros2 run drone check_lap --sitl --api
ros2 run drone mission1 --sitl --api-arm --max-laps 1
```

## Obstacle avoidance in Gazebo

Ported from drone-2026 `oa_bridge/README_GAZEBO.md`. The OA core is unchanged; the node
now reaches PX4 through the agent.

Terminal 1:

```sh
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lidar_2d
gz topic -l | grep scan        # copy the lidar topic
```

Terminal 2:

```sh
ros2 launch drone oa_gazebo.launch.py \
    target_x:=20.0 target_y:=0.0 target_z:=3.0 \
    gz_lidar_topic:=<topic from gz topic -l>
```

This starts the agent, the Gazebo → ROS 2 lidar bridge, and `oa_node`, which arms,
takes off, then avoids obstacles toward the target.

Useful `oa_node` flags: `--beam-stride`, `--max-range`, `--voxel-size`,
`--scan-timeout`, `--brake-distance`, `--no-land` (run `ros2 run drone oa_node --help`).

## Coordinate frames

Every getter, setpoint and target in `drone.px4` and the mission code is **ENU**:
x = East, y = North, z = Up, relative to PX4's local origin. PX4's own topics are
NED; `src/drone/drone/px4/frames.py` converts between them. `target_x:=20` means
20 m East of the local origin.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `Failed to connect to PX4` | Is SITL running? Is the agent running (or was `--no-agent` passed by mistake)? `ros2 topic list \| grep fmu` |
| Topics listed but a callback never fires | QoS mismatch (use `drone.px4.qos.PX4_QOS`) or px4_msgs/firmware mismatch (e.g. `vehicle_status` vs `vehicle_status_v1`) |
| OFFBOARD rejected | Setpoints not flowing first, or no valid local position yet (wait for EKF) |
| Arms but won't move | `ros2 topic hz /fmu/in/offboard_control_mode` should be ~10 Hz |
| `obstacles=0` always, `scans=0` | Wrong `gz_lidar_topic`; re-run `gz topic -l \| grep scan` and `ros2 topic echo /scan` |
| Takeoff times out | Local position invalid; `ros2 run drone check_telemetry --sitl` |
