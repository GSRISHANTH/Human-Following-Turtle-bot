# Vision-Based Human-Following Robot: Complete Setup & Operating Guide

An end-to-end intelligent human-following mobile robot using a **distributed architecture**:
- **Laptop (AI & Intelligence Hub)**: Runs all computer vision, MediaPipe pose tracking, multi-template HSV person Re-ID, obstacle corridor detection, directional search rotation, and the interactive web control dashboard.
- **TurtleBot3 Burger + Raspberry Pi 4 (Hardware & Motor Interface)**: Handles camera hardware video acquisition, streaming, ROS 2 Humble OpenCR motor communication, and local safety watchdog.

---

## Table of Contents
1. [Distributed System Architecture](#1-distributed-system-architecture)
2. [Folder Structure: Which Folder to Go To](#2-folder-structure-which-folder-to-go-to)
3. [Network Setup: How Laptop AI Connects to Raspberry Pi](#3-network-setup-how-laptop-ai-connects-to-raspberry-pi)
4. [Step-by-Step Launch Instructions](#4-step-by-step-launch-instructions)
5. [How to Control the Robot](#5-how-to-control-the-robot)
   - [Natural Skeletal Gestures](#a-natural-skeletal-gestures-hands-free)
   - [Web Dashboard Controls](#b-web-dashboard-controls)
   - [Manual Keyboard & D-Pad Teleoperation](#c-manual-keyboard--d-pad-teleoperation)
   - [Automatic Search & Following Behaviors](#d-automatic-behaviors)
6. [Troubleshooting & Common Issues](#6-troubleshooting--common-issues)

---

## 1. Distributed System Architecture

```
                    ┌──────────────────────────────────────────────┐
                    │      Raspberry Pi 4 (Mounted on TurtleBot)   │
                    │   IP: 10.252.159.140  |  User: ubuntu        │
                    ├──────────────────────────────────────────────┤
                    │ • Hardware Camera Capture (640x480 @ 25 FPS) │
                    │ • UDP video sender (no server, no HTTP)      │
                    │ • UDP command receiver (Port 8888)           │
                    │ • ROS 2 Humble (Domain ID: 30)               │
                    │ • OpenCR 1.0 Motor Driver (/dev/ttyACM0)     │
                    └──────────────────────┬───────────────────────┘
                                           │
         Video + status (UDP -> laptop :5000)│ ▲ HELLO + velocity (UDP -> Pi :8888)
         JPEG datagrams, newest frame wins │ │ V:<vx>,W:<wz>,S:<seq> at 30 Hz
                                           ▼ │
                    ┌──────────────────────────────────────────────┐
                    │        Laptop Processing & Control Station    │
                    │   Windows / Linux  |  Flask Web App: 8000    │
                    ├──────────────────────────────────────────────┤
                    │ • MediaPipe 33-Point Skeletal Pose           │
                    │ • Multi-Template HSV Person Re-ID            │
                    │ • Active Directional Search Routine          │
                    │ • Visual Floor Obstacle Corridors (Sobel)    │
                    │ • Slew-Rate Acceleration Profiling           │
                    │ • Interactive Dual-Feed Web Dashboard        │
                    └──────────────────────────────────────────────┘
```

---

## 2. Folder Structure: Which Folder to Go To

```
laptop/     Vision, gestures, safety, and the UDP command sender. Runs on the laptop.
pi/         Camera, UDP bridge, and ROS 2 /cmd_vel. Copy this folder to the Raspberry Pi.
hil/        Hardware-in-the-loop Burger plant. Runs on the laptop while the battery charges.
tests/      State-machine checks and the kinematic plant check.
templates/  Dashboard page.
static/     Dashboard CSS and JavaScript.
```

### A. On the Laptop (AI Station)
Open PowerShell in this project folder.

| File | Role |
|---|---|
| `app.py` | Starts the laptop dashboard (`laptop/app.py`). |
| `run_dashboard.bat` | Same launch, double-click on Windows. |
| `run_hil.bat` | HIL bench: webcam + simulated Burger. Real motors stay off. |
| `laptop/config.py` | `PI_IP`, follow distance (1.20 m), speed limits, ports. |
| `laptop/cv_pipeline.py` | Pose, Re-ID, distance, search rotation, HUD. |
| `laptop/gesture_recognizer.py` | `FOLLOW_ME`, `WAIT`, `RELEASE`, `STOP_HALT`. |
| `laptop/appearance_model.py` | Multi-template HSV owner profile. |
| `laptop/obstacle_avoidance.py` | Left / center / right floor corridors. |
| `laptop/robot_controller.py` | Slew-rate limiter and UDP packets to port 8888. |
| `laptop/video_streamer.py` | MJPEG client for the Pi (or the virtual Pi). |
| `tests/test_system.py` | 16-scenario checks for gestures, identity, and safety. |

---

### B. On the Raspberry Pi (Robot Side)
```bash
ssh ubuntu@10.252.159.140
# password: turtlebot
```

Copy the `pi/` folder onto the Pi (for example `/home/ubuntu/pi`) and start it from that folder.

| File | Role |
|---|---|
| `pi/start_robot.sh` | Sources ROS 2 Humble, brings up OpenCR, enables torque, launches the companion node next to the script. |
| `pi/pi_turtlebot_node.py` | UDP only: sends camera JPEGs to the laptop, receives velocity on 8888, forwards to ROS 2 `/cmd_vel`. Hosts nothing. `--no-motors`, `--raw-mjpeg`. |
| `pi/tb_proto.py` | UDP wire format shared by the Pi, the laptop and the HIL bench. Copy it with the node. |
| `~/turtlebot3_ws/` | ROS 2 Humble TurtleBot3 workspace on the Pi. |

---

## 3. Network Setup: How Laptop AI Connects to Raspberry Pi

The laptop and Raspberry Pi communicate over local Wi-Fi.

### Step 1: Ensure Same Wi-Fi Network
Both your laptop and the Raspberry Pi must be connected to the exact same Wi-Fi network (or laptop mobile hotspot).

### Step 2: Verify or Find the Raspberry Pi IP Address
- **Option 1 (From Pi terminal)**:
  ```bash
  hostname -I
  ```
  *(Example output: `10.252.159.140`)*
- **Option 2 (From Laptop PowerShell)**:
  ```powershell
  arp -a | Select-String "10.252.159"
  ```

### Step 3: Set the IP in `laptop/config.py`
`PI_IP` defaults to `10.252.159.140`. Change it there, or set it only for one run:

```powershell
$env:PI_IP = "10.252.159.140"
python app.py
```

### Communication Protocols & Ports
1. **Handshake (Laptop -> Pi)**: the laptop sends `HELLO` every 0.5 s from its video socket (UDP 5000) to `<PI_IP>:8888`. The Pi replies to that address, so the Pi needs no laptop IP and nothing listens for HTTP on it.
2. **Camera Feed (Pi -> Laptop)**: UDP datagrams (<= 1200 B) carrying one JPEG per frame, plus a 2 Hz status packet (ROS mode, subscribers, camera FPS). A frame missing a datagram is dropped, so loss never stalls the stream.
3. **Motor Velocity Commands (Laptop -> Pi)**: UDP `<PI_IP>:8888`, text `V:<m/s>,W:<rad/s>,S:<seq>` at 30 Hz. Allow UDP 5000 inbound for Python in Windows Firewall if no video arrives.
4. **Dashboard**: video is forwarded untouched to the browser, the pose is drawn on a canvas, and telemetry is pushed over Server-Sent Events. The Link Health card shows video FPS, pose FPS, frame age, bitrate, drops and the Pi's own status.
5. **Safety Watchdog**:
   - The Raspberry Pi enforces a **0.60-second hardware watchdog**. If communication from the laptop drops for more than 600 ms, the Pi automatically zeroes motor speeds and stops the robot.

---

## 4. Step-by-Step Launch Instructions

### Pre-Flight Hardware Checks:
1. **Battery**: Check that the TurtleBot LiPo battery (11.1V – 12.6V) is securely connected to the OpenCR board.
2. **Power Switch**: Turn ON the physical power switch on the OpenCR board (small switch next to power connector). The LEDs will light up.

---

### Step 1: Start the Raspberry Pi Services
Open a terminal on your laptop and SSH into the Pi:
```bash
ssh ubuntu@10.252.159.140
```
*(Enter password `turtlebot`)*

Run the complete bringup script from the copied `pi` folder:
```bash
bash /home/ubuntu/pi/start_robot.sh
```
You will see:
```text
[TurtleBot] Starting OpenCR hardware bringup...
[TurtleBot] Motor torque enabled.
[TurtleBot] Starting AI Companion Bridge (Camera + /cmd_vel)...
 * Running on http://0.0.0.0:5000/ (Press CTRL+C to quit)
```
*(Keep this terminal open while operating the robot).*

---

### Step 2: Start the Laptop AI Engine & Web Dashboard
On your laptop, open PowerShell in this project folder:
```powershell
py -3.11 app.py
```
Or double-click `run_dashboard.bat`.

You will see:
```text
[CVPipeline] MediaPipe Pose ready.
[CVPipeline] OpenCV HOG multi-person detector ready.
Connecting to Raspberry Pi stream at: http://10.252.159.140:5000/video_feed
Opening Web Dashboard on: http://localhost:8000
```

### Hardware-in-the-loop while the battery charges
The real OpenCR motors are not required for software work. `run_hil.bat` (or `py -3.11 hil/run_hil.py`) starts a virtual Pi on this PC:

- Camera: this laptop's webcam, on `http://127.0.0.1:5001/video_feed`
- Motors: a TurtleBot3 Burger differential-drive plant (same wheel update as ROBOTIS [`turtlebot3_fake_node`](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble/turtlebot3_fake_node/src/turtlebot3_fake_node.cpp)), drawn at `http://127.0.0.1:5001/`
- Commands: the same UDP `V:...,W:...` packets the real Pi accepts on port 8888

The vision dashboard is still `http://localhost:8000`. Stand in front of the laptop camera. The robot on the floor does not move.

On an Ubuntu PC with ROS 2 Humble, the official full-sensor simulator is separate and does not speak this project's UDP bridge:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
# sensors (LiDAR, IMU, camera) need Gazebo instead:
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

---

### Step 3: Open the Web Dashboard
Open your web browser (Chrome, Edge, or Firefox) and navigate to:
```
http://localhost:8000
```
- **Feed 1 (Left)**: Live raw video streamed directly from the Raspberry Pi camera.
- **Feed 2 (Right)**: AI stream displaying MediaPipe 33-point skeletal pose, target reticle, Re-ID confidence percentage, and visual floor obstacle corridors.

---

## 5. How to Control the Robot

The system offers three ways to interact with the robot: natural gestures, web buttons, and manual teleoperation.

---

### A. Natural Skeletal Gestures (Hands-Free)

The laptop uses MediaPipe pose landmarks to recognize intentional human gestures without any keyboard or mouse input:

| Gesture | Physical Pose | System Action | Dashboard Feedback |
|---|---|---|---|
| **`FOLLOW_ME`** | **Raise one hand high above your head/nose** (left or right hand). | • **If no owner is enrolled**: Locks onto you as the exclusive owner.<br>• **If already enrolled**: Starts / resumes autonomous following. | Neon green reticle, `★ OWNER MATCHED 100% \| FOLLOWING` |
| **`WAIT`** | **Hold open palm / forearm upright at chest/shoulder level** (stop palm facing camera). | • Immediately pauses autonomous following.<br>• Robot enters `WAIT_FOR_OWNER_MOVEMENT` and stays completely stationary. | Cyan HUD: `★ OWNER NEARBY \| WAITING FOR OWNER TO MOVE` |
| **`RELEASE`** | **Deliberately cross both arms in an 'X' over your chest** (wrists cross each other). | • Releases owner identity lock and deletes profile.<br>• Robot halts and returns to `IDLE` standby mode. | Blue HUD: `IDLE: NO OWNER LOCKED \| RAISE HAND TO ENROLL` |
| **`STOP_HALT`** | **Raise both hands high above both shoulders simultaneously**. | • Instant Emergency Safety Stop override.<br>• Cuts motor drive immediately. | Flashing Red banner: `EMERGENCY STOP OVERRIDE` |

---

### B. Web Dashboard Controls

Under Feed 1 and Feed 2 on `http://localhost:8000`, the tactical control panel includes:

- **`★ LOCK / ENROLL OWNER`**: Locks the person currently in front of the camera as the designated owner, extracting an immutable HSV color biometric profile.
- **`▶ START FOLLOWING`**: Commands the robot to follow the locked owner autonomously.
- **`⏸ PAUSE`**: Pauses following without releasing the owner profile. The robot stays stopped.
- **`✖ RELEASE OWNER`**: Clears the stored owner biometric profile and puts the robot in standby.
- **`E-STOP` (Large Red Button)**: Global emergency stop switch. Halts all motors instantly.

---

### C. Manual Keyboard & D-Pad Teleoperation

If you want to manually drive the TurtleBot:
1. On the dashboard, switch the **Robot Mode** toggle from `AUTO` to `MANUAL`.
2. Use the on-screen virtual D-Pad or your laptop keyboard:
   - **`W` / `Up Arrow`**: Drive forward
   - **`S` / `Down Arrow`**: Reverse
   - **`A` / `Left Arrow`**: Rotate left
   - **`D` / `Right Arrow`**: Rotate right
   - **`Spacebar`**: Instant brake / stop

---

### D. Automatic Behaviors

#### 1. Smooth Follow-and-Wait Logic (No Hunting)
- **Target Distance**: $1.20\text{ m}$.
- **Approaching Stop**: When the owner stops and distance drops to $\le 1.10\text{ m}$ (`STOP_DISTANCE`), the robot decelerates smoothly to $v_x = 0.0$ and enters `WAIT_FOR_OWNER_MOVEMENT`.
- **Indefinite Hold**: The robot will remain stationary indefinitely while the owner stands nearby. It will **never** search for or switch to someone else.
- **Resume Hysteresis**: When the owner steps away and distance exceeds $\ge 1.35\text{ m}$ (`RESUME_DISTANCE`) for 3 consecutive frames, the robot automatically resumes following.

#### 2. Active Directional Search Rotation (When Going Out of Screen)
- When the owner walks out of the camera frame, the robot remembers whether you exited to the **LEFT** or to the **RIGHT**.
- **Phase 1 (0.6s)**: Brief hesitation to allow Kalman trajectory prediction.
- **Phase 2 (up to 6.0s)**: The robot begins a slow, safe in-place rotation ($\omega_z = 0.35\text{ rad/s}$) toward the exact side you exited.
- **Bystander Rejection**: If unfamiliar people are visible during the rotation, the robot ignores them because their HSV signature does not match the locked owner.
- **Re-Acquisition**: The moment you re-enter the camera field of view, the owner match score triggers ($\ge 0.40$), search mode terminates, and smooth following resumes immediately!

---

## 6. Troubleshooting & Common Issues

### Issue 1: Feed 1 Shows "Disconnected" or Camera Won't Load
- **Cause**: No UDP video is arriving from the Pi (node not running, wrong `PI_IP`, or UDP 5000 blocked on the laptop).
- **Fix**:
  1. Test network connectivity from PowerShell:
     ```powershell
     ping 10.252.159.140
     ```
     On the Pi, `tail ~/pi_node.log` should show `Video peer: <laptop>:5000`.
  2. If it fails, check if the Pi's Wi-Fi IP changed (`hostname -I` on Pi) and update `PI_IP` in [`config.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems/for%20Robotics/Project/human-following-laptop/config.py).
  3. Ensure `start_robot.sh` is running on the Pi.

---

### Issue 2: Motors Do Not Rotate
- **Cause**: OpenCR board is unpowered, motor torque is disabled, or UDP commands are blocked.
- **Fix**:
  1. Check the physical power switch on the OpenCR board.
  2. Test motor response directly on the Pi:
     ```bash
     ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"
     ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
     ```
  3. Check battery voltage: If the OpenCR beeps continuously, the LiPo battery voltage has dropped below $11.0\text{V}$ and must be recharged.

---

### Issue 3: Feed 2 Is Frozen
- **Cause**: Background processing thread encountered an exception.
- **Fix**:
  - The latest code includes automatic exception recovery in `processing_worker()`. If Feed 2 ever pauses, simply refresh your browser window (`Ctrl + F5`).

---

### Issue 4: Robot Loses Owner in Strong Lighting Changes
- **Cause**: Extreme direct sunlight altering clothing HSV appearance.
- **Fix**:
  - Re-enroll the owner by raising one hand (`FOLLOW_ME` gesture) or clicking **★ LOCK / ENROLL OWNER** on the dashboard. The system will capture updated multi-view templates.

---

### Issue 5: Run Automated System Verification Suite
To verify that all 16 state transitions, safety interlocks, and gesture classifiers are functioning correctly:
```powershell
python test_system.py
```
*(All 16 unit tests should report `OK`).*

---

## 7. LiDAR, EKF and PID

- **LiDAR**: LDS-02 on `/scan` (`LDS_MODEL=LDS-02`, started by `ros2 launch turtlebot3_bringup robot.launch.py`). The Pi node bins it to 180 x 2 degrees plus `/odom` and sends it as one UDP datagram at about 8 Hz. It only reads; it never moves the robot.
- **Dashboard LiDAR map**: live scan, odometry-registered occupancy grid (dead reckoning, so it drifts; press CLEAR MAP to reset), avoidance sectors, and the EKF person marker with a velocity arrow.
- **Avoidance**: `laptop/lidar.py` finds the nearest return ahead/left/right with the followed person masked out, slows and steers around obstacles, and `RobotController` hard-stops forward motion if anything is within 0.20 m dead ahead. The camera corridors take over if the scan goes stale.
- **Fusion**: camera pinhole bearing and range (`CAMERA_HFOV_DEG`) are cross-checked against the LiDAR range along that bearing. The LiDAR range is trusted and slowly recalibrates the camera's size-based distance (`vision_scale`).
- **EKF** (`laptop/ekf_person.py`): state is the person's position and ground velocity in the robot frame, polar (range, bearing) measurements, robot ego-motion removed in the prediction. It supplies the velocity, the lead point the robot steers toward, and the predicted direction used to start a search.
- **PID** (`laptop/follow_controller.py`): distance PID plus feed-forward on the person's speed, bearing PID, derivative terms from the EKF rates, anti-windup. Gains are in `Config.PID_*` and were tuned in `tests/test_follow_sim.py`.
- **Motor command format**: `geometry_msgs/Twist` on `/cmd_vel` (Humble), `linear.x` m/s and `angular.z` rad/s, clamped to the Burger's 0.22 m/s and 1.4 rad/s on the Pi.
- **Camera**: it must see people. Tilt it level or slightly down; distance and bearing assume a roughly horizontal optical axis.

### Identity, localisation and planning

- **Re-ID gate** (`CVPipeline._accept_owner`): while tracking, a candidate must score at least 75% of what the owner normally scores *and* sit within a few sigma of where the EKF has them. After any loss (left the view, occluded, program restart) the owner is only re-accepted after `REACQ_CONFIRM_FRAMES` consecutive frames above `max(REACQ_THRESHOLD, 85% of the owner's usual score)` at a consistent place. A stranger who walks in as the owner walks out is logged as `REID_REJECTED` and ignored. The Owner card shows the live match, the threshold and the confirmation progress. Appearance is a colour and proportion signature, so a stranger in near-identical clothes can still score high; raise `REACQ_THRESHOLD` if that happens.
- **LiDAR at 1 degree**, clustered into objects. The camera's person is matched to a person-sized cluster and that cluster's centroid is the LiDAR measurement fed to the EKF. On the map, the person is green, objects that appeared in space seen free before are `MOVING`, and everything else is a static obstacle or wall.
- **Localisation**: `LidarMap` scan-matches each scan against the map built so far (correlative matching over +/-10 cm and +/-4 degrees, damped) and keeps an odom-to-map correction, so wheel-odometry drift is corrected instead of smearing the map.
- **Path planning** (`laptop/navigation.py`): A* on the inflated occupancy grid to a goal `TARGET_DISTANCE` short of the person. If the straight line is clear the robot drives at the person; if not, it steers at a waypoint 0.6 m along the path (`DETOUR`), and stops with `NO_PATH` if none exists. The path is drawn in magenta on the map.
