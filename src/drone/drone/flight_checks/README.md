# The Flight Checks Verification Ladder

This folder contains the **9-step testing ladder** used to verify the companion computer software stack, sensor telemetry, and flight mechanics.

---

## 1. The Philosophy of the Ladder (Escalating Risk Model)

Never skip steps on physical hardware. The ladder is deliberately organized into an **escalating risk model**:

| Step | Script | Execution Risk | Propellers State | Core Objective |
| :---: | :--- | :--- | :---: | :--- |
| **1** | `check_link` | **Zero** | Either | Verifies `MicroXRCEAgent` connectivity and FMU message publishing. |
| **2** | `check_telemetry` | **Zero** | Either | Validates sensor health, GPS fix, EKF state, and battery telemetry. |
| **3** | `check_setpoints` | **Zero** | Either | Dry-runs ENU $\leftrightarrow$ NED coordinate conversions without motor commands. |
| **4** | `check_offboard` | **Zero** | Either | Validates the 10 Hz `OffboardControlMode` heartbeat and mode transitions. |
| **5** | `check_arm` | **Low** | ⚠️ **PROPS OFF** | Verifies motor arming and disarming interlocks. |
| **6** | `check_hover` | **Medium** | Props On (Outdoor) | First autonomous takeoff, 3m hover hold, and auto-landing. |
| **7** | `check_goto_gps` | **Medium** | Props On (Outdoor) | Flies to a single GPS coordinate with absolute position setpoints. |
| **8** | `check_gps_movement` | **High** | Props On (Outdoor) | Flies between sequential waypoints while facing the direction of travel. |
| **9** | `check_lap` | **High** | Props On (Outdoor) | Executes a complete polygonal perimeter lap and Returns-to-Launch (RTL). |

---

## 2. Testing Environments & Safety Rules

### Environment A: Gazebo Simulation (SITL)
* **Link flag:** `--sitl` (Connects over UDP to `127.0.0.1:8888`).
* **Arming:** Pass `--api` to command arming from software.
* **Safety:** Virtual propellers cannot injure anyone; code-commanded arming is permitted.

### Environment B: Physical Bench Test (Lab / Desktop)
* **Link flag:** `--serial /dev/ttyTHS1` (or `--udp 8888`).
* **Propellers:** 🛑 **MANDATORY: REMOVE ALL PROPELLERS BEFORE BENCH TESTING.**
* **Arming:** Use RC switch, or bench `--api` only when blades are physically off the drone.

### Environment C: Airfield Flight (Outdoor)
* **Link flag:** `--serial` (or `--udp` depending on hardware wiring).
* **Propellers:** Installed.
* **Arming:** **SOFTWARE ARMING (`--api`) IS STRICTLY FORBIDDEN.** The safety pilot must arm exclusively with the physical RC transmitter switch.
* **Failsafe:** The pilot must keep their finger on the RC flight mode switch. Flipping to **POSCTL** (Position Control) immediately cuts off companion computer control and gives manual control back to the pilot.

---

## 3. Step-by-Step Instructions

### Step 1: `check_link` — Bridge Connectivity
Confirms that `MicroXRCEAgent` starts, reaches PX4, and detects active uORB topic streams.

* **Simulation:**
  ```bash
  ros2 run drone check_link --sitl
  ```
* **Hardware:**
  ```bash
  ros2 run drone check_link --serial /dev/ttyTHS1
  ```
* **Success Output:**
  ```text
  [CHECK] PX4 publishers visible through the agent:
      /fmu/out/vehicle_status_v1                 publishers=1
      /fmu/out/vehicle_local_position_v1         publishers=1
      /fmu/out/battery_status                    publishers=1
  [MAIN] ✓ Check passed
  ```
* **Failure Troubleshooting:**
  - If `publishers=0`: Verify baud rate matches (`--baud 921600`) and the flight controller is powered.
  - If `MicroXRCEAgent not found`: Install `Micro-XRCE-DDS-Agent` v2.4.2 to `/usr/local/bin`.

---

### Step 2: `check_telemetry` — Sensor & EKF Health
Prints all live telemetry received from PX4 without commanding any motion.

* **Command:**
  ```bash
  ros2 run drone check_telemetry --sitl --duration 10
  ```
* **Verification Checks:**
  - **Altitude ($z$):** Physically lift the drone $\rightarrow$ $z$ (Up) must increase.
  - **North ($y$):** Walk the drone North $\rightarrow$ $y$ (North) must increase.
  - **Heading (Yaw):** Point nose North $\rightarrow$ Yaw is $\approx +90^\circ$; point East $\rightarrow$ Yaw is $\approx 0^\circ$.
* **Success Output:**
  ```text
  [TELEMETRY] Basic Status
  Connected:     True
  Position (ENU): E=0.02m, N=-0.01m, U=0.00m
  Yaw (ENU):     89.8°
  Battery:       98% (16.20V, 0.40A)
  GPS Receiver:  fix_type=3, satellites=14
  ```

---

### Step 3: `check_setpoints` — Setpoint Math Sanity Check
Publishes position and velocity setpoints in ENU and verifies that they correctly translate to aviation NED on the wire. **The vehicle remains disarmed.**

* **Command:**
  ```bash
  ros2 run drone check_setpoints --sitl
  ```
* **In a second terminal, verify coordinate translation:**
  ```bash
  ros2 topic echo /fmu/in/trajectory_setpoint
  ```
  *When script logs `Up (positive z is up in ENU): 1.0 m/s`, the topic must show `velocity: [0.0, 0.0, -1.0]` (negative $z$ is up in NED).*

---

### Step 4: `check_offboard` — Heartbeat Handshake
Starts the 10 Hz background heartbeat, commands PX4 into `OFFBOARD` mode, verifies PX4 holds it for 5 seconds, and cleanly exits. **The vehicle remains disarmed.**

* **Command:**
  ```bash
  ros2 run drone check_offboard --sitl --hold 5
  ```
* **Success Output:**
  ```text
  [CHECK] ✓ OFFBOARD active
  [CHECK] Maintaining OFFBOARD for 5 seconds (background heartbeat publishing)...
  [CHECK] ✓ OFFBOARD stable for 5 seconds
  [MAIN] ✓ Check passed
  ```

---

### Step 5: `check_arm` — Motor Interlock Test
Validates the arming sequence. Enters OFFBOARD mode, prints pre-arm sensor diagnostics, arms the motors, and immediately disarms.

* **Simulation:**
  ```bash
  ros2 run drone check_arm --sitl --api
  ```
* **Hardware (⚠️ PROPS REMOVED):**
  ```bash
  ros2 run drone check_arm --serial /dev/ttyTHS1
  # Flip physical RC arm switch when prompted
  ```
* **Success Output:**
  ```text
  === PRE-ARM DIAGNOSTICS ===
  Battery Voltage: 16.20V
  GPS Satellites: 14
  GPS Fix: 3
  Home Set: Yes
  ===========================
  [CHECK] ✓ Vehicle armed
  [MAIN] ✓ Check passed
  ```

---

### Step 6: `check_hover` — First Flight & Altitude Hold
The first airborne flight check. Arms, enters OFFBOARD mode, commands takeoff to the designated altitude, hovers for 10 seconds, lands, and disarms.

* **Simulation:**
  ```bash
  ros2 run drone check_hover --sitl --api --altitude 3 --hover-seconds 10
  ```
* **Hardware (Outdoor Field):**
  ```bash
  ros2 run drone check_hover --serial --altitude 2 --hover-seconds 5
  ```
* **Parameters:**
  - `--altitude`: Height to climb in meters (default 5.0m).
  - `--hover-seconds`: Duration to hold position (default 10s).

---

### Step 7: `check_goto_gps` — Single Waypoint Navigation
Takes off, anchors a GPS target coordinate to the drone's local ENU coordinate frame, streams absolute position setpoints at 10 Hz until arrival within 1.0 meter tolerance, and auto-lands.

* **Simulation:**
  ```bash
  ros2 run drone check_goto_gps --sitl --api --lat 47.39785 --lon 8.54573 --alt 10
  ```
* **Hardware:**
  ```bash
  ros2 run drone check_goto_gps --serial --lat <TARGET_LAT> --lon <TARGET_LON> --alt 5
  ```

---

### Step 8: `check_gps_movement` — Trajectory Tracking
Takes off, commands travel toward a target GPS waypoint, and dynamically points the drone's nose (yaw) in the direction of flight.

* **Simulation:**
  ```bash
  ros2 run drone check_gps_movement --sitl --api --target 47.39785,8.54573,5
  ```
* **Hardware:**
  ```bash
  ros2 run drone check_gps_movement --serial --target <LAT>,<LON>,<ALT_AGL>
  ```

---

### Step 9: `check_lap` — Full Perimeter Flight Loop
Executes a multi-waypoint perimeter lap using smooth velocity setpoints (fast on straightaways, slowing on corner approach) and finishes with an autonomous Return-To-Launch (RTL).

* **Simulation:**
  ```bash
  ros2 run drone check_lap --sitl --api --alt 5
  ```
* **Hardware:**
  ```bash
  ros2 run drone check_lap --serial --alt 5
  ```
* **Configuration:** Edit `LAP_WAYPOINTS` inside `check_lap.py` with your airfield GPS coordinates before flying.

---

## 4. Shared Command-Line Flags

Every check script accepts the common link arguments defined in [`src/drone/drone/px4/agent.py`](file:///Users/benmochen/SynologyDrive/Programming/McGill%20Robotics/drone-2027/src/drone/drone/px4/agent.py):

| Flag | Argument | Description |
| :--- | :--- | :--- |
| `--sitl` | None | Starts `MicroXRCEAgent` on UDP port `8888` (Simulation). |
| `--serial` | `[DEVICE]` | Connects over serial UART (default `/dev/ttyTHS1`). |
| `--baud` | `[BAUD]` | Serial baud rate (default `921600`). |
| `--udp` | `[PORT]` | Connects over Ethernet UDP (default `8888`). |
| `--no-agent` | None | Bypasses launching the agent if one is already running in another terminal. |
| `--api` | None | Allows software-commanded arming (only allowed with `--sitl`). |

---

## 5. Emergency Failsafe Procedures

During any physical outdoor flight test (Steps 6–9):

1. **Manual Flight Mode Takeover:**
   Flip the designated RC flight mode switch on your transmitter from `OFFBOARD` to **`POSCTL` (Position Control)** or **`STABILIZED`**. PX4 will immediately drop companion computer control and respond 100% to pilot stick inputs.
2. **Return-To-Launch (RTL):**
   Flip the RC RTL switch. The drone will climb to safe return altitude, fly straight back to the home launch position, and land.
3. **Emergency Kill Switch:**
   In an unavoidable collision or flyaway emergency, pull the physical RC Kill Switch (`RC_MAP_KILL_SW`). This instantly cuts motor power.
