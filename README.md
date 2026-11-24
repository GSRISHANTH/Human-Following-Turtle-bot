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
                    │   IP: 10.242.147.140  |  User: ubuntu        │
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
ssh ubuntu@10.242.147.140
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
  *(Example output: `10.242.147.140`)*
- **Option 2 (From Laptop PowerShell)**:
  ```powershell
  arp -a | Select-String "10.242.147"
  ```

### Step 3: Set the IP in `laptop/config.py`
`PI_IP` defaults to `10.242.147.140`. Change it there, or set it only for one run:

```powershell
$env:PI_IP = "10.242.147.140"
python app.py
```

### Communication Protocols & Ports
1. **Handshake (Laptop -> Pi)**: the laptop sends `HELLO` every 0.5 s from its video socket (UDP 5000) to `<PI_IP>:8888`. The Pi replies to that address, so the Pi needs no laptop IP and nothing listens for HTTP on it.
2. **Camera Feed (Pi -> Laptop)**: UDP datagrams (<= 1200 B) carrying one JPEG per frame, plus a 2 Hz status packet (ROS mode, subscribers, camera FPS). A frame missing a datagram is dropped, so loss never stalls the stream.
3. **Motor Velocity Commands (Laptop -> Pi)**: UDP `<PI_IP>:8888`, text `V:<m/s>,W:<rad/s>,S:<seq>` at 30 Hz. Allow UDP 5000 inbound for Python in Windows Firewall if no video arrives.
4. **Dashboard**: video is forwarded untouched to the browser, the pose is drawn on a canvas, and telemetry is pushed over Server-Sent Events. The Link Health card shows video FPS, pose FPS, frame age, bitrate, drops and the Pi's own status.
5. **Safety Watchdog**:
   - Two watchdogs. The Raspberry Pi stops the wheels if no velocity packet arrives for **0.35 s** (`CMD_TIMEOUT`, about ten missed 30 Hz packets). On the laptop, `RobotController` zeroes the commanded velocity if the vision pipeline has not refreshed it for **0.60 s**.

---

## 4. Step-by-Step Launch Instructions

### Install Dependencies (once)
Laptop (Python 3.10 - 3.14; tested on 3.14.6):
```powershell
py -3.14 -m pip install -r requirements.txt
py -3.14 -m unittest discover tests     # expect: Ran 128 tests ... OK
```
Raspberry Pi (ROS 2 Humble supplies rclpy and the message packages):
```bash
python3 -m pip install --user -r ~/pi/requirements.txt
```

### Pre-Flight Hardware Checks:
1. **Battery**: Check that the TurtleBot LiPo battery (11.1V – 12.6V) is securely connected to the OpenCR board.
2. **Power Switch**: Turn ON the physical power switch on the OpenCR board (small switch next to power connector). The LEDs will light up.

---

### Step 1: Start the Raspberry Pi Services
Open a terminal on your laptop and SSH into the Pi:
```bash
ssh ubuntu@10.242.147.140
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
Connecting to Raspberry Pi stream at: http://10.242.147.140:5000/video_feed
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
     ping 10.242.147.140
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

### Close range, lost-person search, speeds

- **Close range**: when you are too close for the low camera to see more than your legs, the person-sized LiDAR cluster at the EKF's expected position keeps the track alive (status `OWNER (LIDAR)`, up to `LIDAR_TRACK_MAX_SEC`). The robot stops and waits at the follow distance and resumes when you walk 15 cm beyond it, instead of searching. The distance is set with the dashboard slider (floor `MIN_FOLLOW_DISTANCE`, 0.6 m).
- **Lost for real**: the EKF coasts your last position and velocity (velocity decays), the robot turns toward where you were heading, then sweeps wider around that direction for `SEARCH_MAX_DURATION_SEC`, using LiDAR to avoid rotating into something.
- **Re-ID**: while the track is alive, position continuity decides and your appearance from any side is learned (only when you are alone in the picture). After a loss, appearance must clear `max(REACQ_THRESHOLD, 0.6 x your usual score)` for a few frames, fewer and easier near where you were heading.
- **Calibration**: the camera's distance scale and its bearing offset are estimated against the LiDAR while both see you. Frames are timestamped on arrival and the EKF is advanced to the present before control (about 100 ms of latency).
- **Speeds**: forward is capped at 0.22 m/s, the Burger's hardware limit. Turning is up to 2.0 rad/s (hardware 2.84), accelerations 1.6 m/s^2 and 8 rad/s^2.
- **Hardware movement check**: `python tests/hw_move_check.py` (dashboard stopped) drives a few centimetres and compares odometry with the LiDAR.
- **Power**: if the Pi reboots or the LiDAR's USB drops when the motors run, check the battery and the Pi's 5 V supply. The Pi node enables motor torque itself and retries until the driver confirms.

### Search sequence (verified 360 degrees)

`CVPipeline._search_command` runs three phases once the owner is lost: **PREDICTED** (face the EKF's predicted bearing), **SWEEP** (swing out to +/-110 degrees around it for `SEARCH_SWEEP_SEC`), then **SCAN** (keep turning the same way, the side the owner left, at `SEARCH_SCAN_SPEED` until the integrated rotation reaches 360 degrees) and stop with `SEARCH_COMPLETE_360`. `tests/test_behaviors.py` drives it with a simulated yaw and asserts every 10-degree heading is visited. On the robot a 360 degree turn at 0.8 rad/s takes 8.4 s and stops about 14 degrees past the circle.

### Tuning on the real robot

`python tests/hw_follow_virtual.py [--gains=current,snappier-turn,...]` runs the real EKF and PID against a virtual person that walks, turns toward the roomier side and stops, while the real wheels, odometry and network latency close the loop. It needs about 2.5 m clear ahead and 0.5 m behind, and aborts if the LiDAR sees anything within 0.55 m ahead. It prints range and bearing error, end distance, peak speeds and turn activity per gain set, then the best one.

---

## 8. Configuration, SLAM and Co-Mapping

### Nothing is hard-coded
Every setting in `laptop/config.py` can be overridden without editing code, in this order:
1. `config.local.json` in the project root (copy `config.example.json`; git ignores the local file).
2. Environment variables `HF_<NAME>`, e.g. `$env:HF_DESIRED_DISTANCE = "1.0"`. `PI_IP` and `VIDEO_PORT` also work bare.

The dashboard's **LIVE SETTINGS** card changes follow distance, speed limits, PID gains, camera pose, Re-ID thresholds and the Pi IP while the robot runs. Every value is range-checked. **APPLY & SAVE** writes the accepted values to `config.local.json`.

### SLAM: three pose sources, best first
| Source | When | How |
|---|---|---|
| `slam` | `SLAM=1 bash start_robot.sh` on the Pi with `ros-humble-slam-toolbox` installed | slam_toolbox publishes TF `map -> odom`. The Pi node (`--slam`) sends `map -> base_footprint` and the laptop uses it as is. |
| `odom` | default: `/odom` from OpenCR | The laptop's scan matcher corrects wheel-odometry drift every scan. |
| `scan` | no `/odom` at all | Scan matching alone, Hector-style, with a constant-velocity prior. |

The laptop matcher (`laptop/lidar.py`) runs in three stages:
1. A likelihood field `exp(-d²/2σ²)` built from the occupancy grid.
2. A multi-resolution correlative search over ±0.30 m and ±15° (Olson's M3RSM pyramid).
3. Gauss-Newton sub-cell refinement.

It takes about 11 ms per scan. People and moving objects are kept out of the map, and the 12 m grid shifts as the robot drives so it never runs off the edge. The map panel shows which source is active and the match score.

### LiDAR ↔ camera co-mapping
`laptop/fusion.py` projects every LiDAR beam into the camera image using the camera pose in the config (`CAM_X_M`, `CAM_HEIGHT_M`, `LIDAR_HEIGHT_M`, `CAM_YAW_DEG`, `CAM_PITCH_DEG`) plus the yaw bias the system learns online:
- **Overlay:** the live view draws the beams as a "laser curtain" coloured by range, with the owner's beams in green.
- **Per-person range:** each person's box is labelled `cam X m | lidar Y m | match Z%`.
- **Association:** the LiDAR person cluster is only accepted if its bearing lies inside that person's image box.

**To calibrate:** stand 2 m in front of the robot. Adjust `CAM_YAW_DEG` until your green LiDAR dots sit on your legs, and `CAM_HEIGHT_M`/`CAM_PITCH_DEG` until the curtain meets the floor-level objects it hits.

### Dashboard panels
- **State machine:** the live state is highlighted, with the reason it is there and how long it has been there.
- **Control loop** (last 30 s, hover for values):
  - range against the set distance and the stop/resume band;
  - linear and angular velocity, commanded (dashed) against actual;
  - owner match score against the threshold.
- **Banners:** VIDEO LOST, STOP LATCHED and TWO LOOK-ALIKES appear over the video.

### Safety behaviour (state machine rules)
- **Who can command:** once an owner is enrolled, only the owner's gestures count. A bystander's FOLLOW ME, WAIT or RELEASE is ignored and logged. Anyone's STOP is honoured.
- **STOP latches:** the robot stays stopped until the owner raises one hand or someone presses START.
- **Look-alikes:** two people who match equally well make the robot hold still. It never searches or guesses.
- **Lost video:** if the camera fails, the Pi says it has no camera, or the link drops, perception stops and the robot halts. The offline test pattern is never treated as a camera frame.
- **Saved profile:** a profile from an earlier run is loaded, but the robot does not move until FOLLOW ME or START.

### HIL bench now includes LiDAR
`run_hil.bat` simulates an LDS-02 scan of a 6 × 4 m furnished room (`hil/world.py`) at 10 Hz with the plant's odometry, so the map, SLAM and co-mapping all run without the robot.

## 9. Hold-to-Follow, Orientation Check and the RViz View

### Hold-to-follow (default; `FOLLOW_REQUIRES_RAISED_HAND`)
| You do | Robot does |
|---|---|
| Hold one hand **above your shoulder** | Follows. It starts after 3 frames with the hand up. |
| Lower your hand | Stops after 0.6 s, then stays put and keeps turning to keep you centred (`AWAITING_FOLLOW_GESTURE`). It never searches. |
| Walk out of the camera's view while the LiDAR still sees you | `LIDAR_GUIDED_SEARCH`: **rotates** toward your LiDAR position to bring you back into view. It never drives blind, and the LiDAR-only track expires after 4 s without camera confirmation. |
| Disappear from both camera and LiDAR | **Rotate to find:** turns at once toward where you went, sweeps wider (±110°), does a full 360°, then stops. Driving to the last-seen spot is optional (`RECOVERY_DRIVE_TO_LAST_SEEN`, off by default). |
| Reappear during the search | The search stops, it centres you and waits for your hand. |
| Both hands up | STOP (latched), from anyone. |

**Following distance adapts:**
- **Formula:** set distance + 0.5 s × your walking speed + up to 0.35 m when tracking confidence is low (camera-only or LiDAR-only ranging, high EKF uncertainty, weak Re-ID).
- **Cap:** at most +0.6 m.
- **On the dashboard:** shown as *Dynamic* on the distance card and as *target* on the range chart.

**Sensor fault:** if the LiDAR stops sending scans for 1.5 s, the robot stops with a SENSOR FAULT banner until scans return.

Only the enrolled owner's hand counts. Untick *Hold-to-follow* in LIVE SETTINGS to go back to gesture-latched following.

### Left/right orientation check (10 s)
Stand about 1.5 m to the robot's **right**:
1. **Live view:** you must appear on the **right** of the picture. If you don't, tick *Camera mirrored*.
2. **Map:** you must appear to the **right** of `base_link` (green Y axis points left). If you don't, tick *LiDAR mirrored*. If you appear rotated instead, set *LiDAR yaw offset*.

While you are tracked, the dashboard also checks this by itself. If the LiDAR keeps finding you on the mirror side of the camera, a **LEFT/RIGHT MISMATCH** banner appears.

### RViz-style map view
The map card is a top-down, RViz-like view in the fixed frame `map` (x right, y up):
- **Displays:** occupancy grid (free light grey, occupied black), 1 m grid, LaserScan coloured by object type, odometry trail, planned path with goal pose, last-seen marker, owner with velocity arrow and 2σ covariance ellipse, `base_link` TF axes, Burger footprint and camera field of view.
- **Controls:** drag to pan, wheel to zoom, double-click to reset; FOLLOW and ROBOT UP toggles; a checkbox per display.
- **Data:** `/api/scene` (10 Hz SSE) and `/api/map.png` (the occupancy grid, with its origin in the response headers).
