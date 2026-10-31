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
                    │ • MJPEG Streaming Server (Port 5000)         │
                    │ • UDP Motor Command Bridge (Port 8888)       │
                    │ • ROS 2 Humble (Domain ID: 30)               │
                    │ • OpenCR 1.0 Motor Driver (/dev/ttyACM0)     │
                    └──────────────────────┬───────────────────────┘
                                           │
         Camera Video Feed (HTTP Port 5000)│ ▲ Velocity Commands (UDP Port 8888)
         http://<PI_IP>:5000/video_feed    │ │ [vx, wz, timestamp, seq]
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

### A. On the Laptop (AI Station)
Open **PowerShell** or **Command Prompt** and navigate to:
```powershell
cd "c:\Studies\7th sem\Embedded Systems for Robotics\Project\human-following-laptop"
```

#### Key Laptop Files:
| File | Role |
|---|---|
| [`app.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/app.py) | **Main entry point**. Runs Flask server, background AI processing loop, video streaming, and REST API. |
| [`config.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/config.py) | **Central settings**. Set `PI_IP`, target distance ($1.20\text{m}$), speeds, thresholds, and ports. |
| [`cv_pipeline.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/cv_pipeline.py) | **Master AI Engine**. Integrates pose, Re-ID, distance estimation, search rotation, and tactical HUD. |
| [`gesture_recognizer.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/gesture_recognizer.py) | Classifies 4 skeletal gestures (`FOLLOW_ME`, `WAIT`, `RELEASE`, `STOP_HALT`). |
| [`appearance_model.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/appearance_model.py) | Multi-template HSV histogram biometric profile, preventing target-swapping. |
| [`obstacle_avoidance.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/obstacle_avoidance.py) | Scans left/center/right floor corridors using Sobel filtering while masking the human. |
| [`robot_controller.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/robot_controller.py) | Packages commands with timestamps, enforces acceleration limits, and sends UDP packets. |
| [`run_dashboard.bat`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/run_dashboard.bat) | One-click Windows batch script to launch [`app.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/app.py). |
| [`test_system.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/test_system.py) | 16-scenario unit test suite verifying state transitions, gestures, and safety layers. |

---

### B. On the Raspberry Pi (Robot Side)
To access the Raspberry Pi from your laptop via SSH:
```bash
ssh ubuntu@10.252.159.140
# When prompted, enter password:
turtlebot
```

Once logged into the Pi, go to the robot workspace:
```bash
cd /home/ubuntu
```

#### Key Raspberry Pi Files:
| File | Role |
|---|---|
| `/home/ubuntu/start_robot.sh` | **One-step bringup script**. Sources ROS 2 Humble, brings up OpenCR, enables torque, and launches companion node. |
| `/home/ubuntu/pi_turtlebot_node.py` | Captures camera frames, hosts MJPEG on port 5000, listens for UDP motor packets on port 8888, and forwards to ROS 2 topic `/cmd_vel`. |
| `/home/ubuntu/turtlebot3_ws/` | ROS 2 Humble TurtleBot3 workspace containing hardware packages. |

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

### Step 3: Set the IP in `config.py` on the Laptop
Open [`config.py`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/config.py) on your laptop. At the top of the file, make sure `PI_IP` matches the Pi's actual IP:
```python
# config.py
PI_IP = "10.252.159.140"  # <-- Update this if the Wi-Fi IP changes
```

### Communication Protocols & Ports
1. **Camera Feed (Pi -> Laptop)**:
   - URL: `http://<PI_IP>:5000/video_feed`
   - Format: Hardware MJPEG stream (640x480, 25 FPS).
2. **Motor Velocity Commands (Laptop -> Pi)**:
   - Protocol: **UDP port 8888** (JSON packet with linear $v_x$, angular $\omega_z$, timestamp, and sequence number).
   - Backup protocol: HTTP POST to `http://<PI_IP>:5000/cmd_vel`.
3. **Safety Watchdog**:
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

Run the complete bringup script:
```bash
bash /home/ubuntu/start_robot.sh
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
On your laptop, open a PowerShell terminal in the project directory:
```powershell
cd "c:\Studies\7th sem\Embedded Systems for Robotics\Project\human-following-laptop"
python app.py
```
*(Alternatively, simply double-click [`run_dashboard.bat`](file:///c:/Studies/7th%20sem/Embedded%20Systems%20for%20Robotics/Project/human-following-laptop/run_dashboard.bat)).*

You will see:
```text
[CVPipeline] MediaPipe Pose ready.
[CVPipeline] OpenCV HOG multi-person detector ready.
Connecting to Raspberry Pi stream at: http://10.252.159.140:5000/video_feed
Opening Web Dashboard on: http://localhost:8000
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
- **Cause**: The laptop cannot reach the Raspberry Pi camera server on port 5000.
- **Fix**:
  1. Test network connectivity from PowerShell:
     ```powershell
     python -c "import urllib.request; print(urllib.request.urlopen('http://10.252.159.140:5000/video_feed', timeout=3).status)"
     ```
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
