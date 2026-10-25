# Autonomous Search-and-Rescue Robot System

An end-to-end autonomous navigation and perception system for search-and-rescue robotics developed for Cal Hacks. The system combines real-time RF/sensor target triangulation, computer vision, triple ultrasonic sensing, and a custom **Streamline Artificial Potential Field (APF)** navigation engine. It is engineered to navigate unknown disaster environments (collapsed structures, multi-layered mazes, and office partitions) without pre-mapped global occupancy grids or wheel encoders.

---

## 📌 Table of Contents
1. [System Architecture & Pipeline](#1-system-architecture--pipeline)
   - [Hardware & Network Overview](#hardware--network-overview)
   - [Physical Pinout Specification (Raspberry Pi 40-Pin Header)](#physical-pinout-specification)
2. [The Navigation Engine](#2-the-navigation-engine)
   - [Why Artificial Potential Fields (APF)?](#why-artificial-potential-fields-apf)
   - [Mathematical Formulation & Streamlines](#mathematical-formulation--streamlines)
   - [Overcoming Classic APF Failure Modes (The Boredom Engine)](#overcoming-classic-apf-failure-modes)
   - [No-Encoder Autonomous Navigation: Why It Works](#no-encoder-autonomous-navigation-why-it-works)
3. [Motor Kinematics (3-Wheel Omni-Directional Drive)](#3-motor-kinematics-3-wheel-omni-directional-drive)
4. [Computer Vision & Sensor Perception](#4-computer-vision--sensor-perception)
   - [Low-Latency JPEG CompressedImage Streaming](#low-latency-jpeg-compressedimage-streaming)
   - [Floor Horizon Canny Edge Segmentation](#floor-horizon-canny-edge-segmentation)
   - [100% Offline Human Victim Detection (YOLOv8 & OpenCV HOG)](#100-offline-human-victim-detection)
   - [Sensor Fusion Envelope](#sensor-fusion-envelope)
5. [Docker & FastDDS Shared Memory Architecture](#5-docker--fastdds-shared-memory-architecture)
6. [Offline LAN Synchronization (`sync_all.ps1`)](#6-offline-lan-synchronization)
7. [Step-by-Step Production Bringup Guide](#7-step-by-step-production-bringup-guide)
8. [Simulation Testing & Visual Scenarios](#8-simulation-testing--visual-scenarios)
9. [ROS 2 Topic Interfaces](#9-ros-2-topic-interfaces)
10. [Repository File Tree](#10-repository-file-tree)

---

## 1. System Architecture & Pipeline

The system is partitioned into a high-performance **Edge Compute Base Station (ASUS Ascent GX10)** and a lightweight **Mobile Robot Client (Raspberry Pi 4)** communicating over local LAN.

```text
[ Mobile Robot: Raspberry Pi 4 (192.168.0.10) ]              [ ASUS Ascent GX10 AI Base Station (192.168.0.232) ]
+-------------------------------------------------+          +------------------------------------------------------+
|                  THE EYES                       |          |                      THE BRAIN                       |
| - Logitech Brio USB Camera (/dev/video0)        |          |  launch_gx10.py (Single-Process Unified Executor)    |
|   -> /image_raw/compressed (JPEG @ 20 FPS)     | ==Wi-Fi==>  VisualWallDetector:                                 |
| - Triple HC-SR04 Rangefinders:                  |          |     * Floor Horizon Canny Edge Obstacle Scan         |
|   -> /ultrasonic_center (Trig 12, Echo 13)      |          |     * Offline Victim Detection (YOLOv8 / HOG)        |
|   -> /ultrasonic_left   (Trig 20, Echo 21)      |          |     * 10Hz Continuous Sensor Fusion -> /obstacle_status|
|   -> /ultrasonic_right  (Trig 5,  Echo 6)       |          |  NavigatorNode:                                      |
+-------------------------------------------------+          |     * Streamline APF Pathfinding + Dead-End Escape   |
                                                             |     * Autonomous Exploration Mode -> /cmd_vel        |
+-------------------------------------------------+          +------------------------------------------------------+
|                  THE BODY                       |                                     |
|  motor_node.py (ROS 2 Subscriber)               | <================Wi-Fi==============+
| - 3-Wheel Kiwi Omni DC Motors (PWM)             |                /cmd_vel (Twist)
|   A: GPIO 17/27, B: GPIO 22/23, C: GPIO 24/25   |
| - 0.35s Safety Watchdog Timer                   |
+-------------------------------------------------+
```

### Hardware & Network Overview
* **Master Workspace**: Windows Laptop (`c:\Users\Huitae\Cal_Hacks`) pushes code via LAN SCP.
* **Robot Host**: Raspberry Pi 4 at `192.168.0.10` (user: `robot1`).
* **AI Workstation**: ASUS Ascent GX10 at `192.168.0.232` (user: `asus`, mounted at `/home/asus/calhacks:/workspace`).
* **Network**: Offline local Wi-Fi router (no internet dependency).

---

### Physical Pinout Specification

To eliminate hardware conflicts between the 3 DC motors and 3 ultrasonic sensors, the Raspberry Pi 40-pin header is assigned as follows:

```text
 Raspberry Pi 40-Pin Header Layout:
 ----------------------------------------------------
 Pin 11 (GPIO 17) : Motor A Forward PWM
 Pin 13 (GPIO 27) : Motor A Reverse PWM
 Pin 15 (GPIO 22) : Motor B Forward PWM
 Pin 16 (GPIO 23) : Motor B Reverse PWM
 Pin 18 (GPIO 24) : Motor C Forward PWM
 Pin 22 (GPIO 25) : Motor C Reverse PWM
 ----------------------------------------------------
 Pin 29 (GPIO 5)  : Right Sensor TRIG
 Pin 30 (GND)     : Ground (Common)
 Pin 31 (GPIO 6)  : Right Sensor ECHO
 Pin 32 (GPIO 12) : Center Sensor TRIG
 Pin 33 (GPIO 13) : Center Sensor ECHO
 Pin 34 (GND)     : Ground (Common)
 Pin 38 (GPIO 20) : Left Sensor TRIG
 Pin 39 (GND)     : Ground (Common)
 Pin 40 (GPIO 21) : Left Sensor ECHO
 ----------------------------------------------------
 Pin 2, 4 (5V)    : Sensor VCC Power
```

---

## 2. The Navigation Engine

### Why Artificial Potential Fields (APF)?
Traditional SLAM stacks (like Nav2 / Global A* / Dijkstra) require pre-built occupancy grids. In sudden disaster deployments:
1. Mapping before moving is too slow for time-critical search and rescue.
2. Global Cartesian grids consume massive memory and CPU cycles.
3. APF operates strictly on instantaneous reactive forces in **$O(1)$ constant time at 10 Hz**, producing fluid, organic motion around debris.

---

### Mathematical Formulation & Streamlines

#### 1. Goal Attraction (The Pull)
Pulls the robot toward the target $(X_{\text{target}}, Y_{\text{target}})$ with terminal arrival deceleration:
$$\text{dist} = \sqrt{(X_{\text{target}} - X_{\text{robot}})^2 + (Y_{\text{target}} - Y_{\text{robot}})^2}$$
$$F_{\text{mag}} = \min(K_{\text{att}} \cdot \text{dist},\; F_{\text{max}})$$
$$\vec{F}_{\text{att, world}} = F_{\text{mag}} \cdot \begin{bmatrix} (X_{\text{target}} - X_{\text{robot}}) / \text{dist} \\ (Y_{\text{target}} - Y_{\text{robot}}) / \text{dist} \end{bmatrix}$$

Transformed into the robot's local body frame (+X = forward, +Y = left) using robot yaw $\theta$:
$$F_{\text{att, robot, x}} = F_{\text{att, world, x}} \cos(\theta) + F_{\text{att, world, y}} \sin(\theta)$$
$$F_{\text{att, robot, y}} = -F_{\text{att, world, x}} \sin(\theta) + F_{\text{att, world, y}} \cos(\theta)$$

#### 2. Steep Canal Embankment Repulsion (The Push)
Obstacle pushback rises exponentially when an object breaches the danger threshold ($d < 0.55\text{m}$):
$$\text{ratio} = \frac{\text{danger\_zone} - d}{\max(d, 0.04)}$$
$$F_{\text{rep}} = \min\left(K_{\text{rep}} \cdot \text{ratio}^{1.8},\; F_{\text{rep, max}}\right)$$
* **Front obstacle**: Pushes backward ($-F_{\text{rep, front}}$ along X).
* **Left obstacle**: Pushes rightward ($-F_{\text{rep, left}}$ along Y).
* **Right obstacle**: Pushes leftward ($+F_{\text{rep, right}}$ along Y).

#### 3. Smooth Streamline Bypass (Vortex Potential)
Classic APF gets stuck facing a flat wall head-on (forward attraction and backward repulsion cancel to 0). We introduce a **tangential streamline potential**:
$$F_{\text{bypass, mag}} = \max(F_{\text{rep, front}} \cdot 1.8,\; 1.4)$$
$$F_{\text{bypass, robot, x}} = (T_{\text{world, y}} \cdot F_{\text{bypass, mag}}) \cdot \sin(\theta) \quad \text{(forward drive once aligned!)}$$
$$F_{\text{bypass, robot, y}} = (T_{\text{world, y}} \cdot F_{\text{bypass, mag}}) \cdot \cos(\theta) \quad \text{(steering torque, drops to 0 when aligned!)}$$

---

### Overcoming Classic APF Failure Modes

| Classic APF Problem | Root Cause | Our Solution |
| :--- | :--- | :--- |
| **Head-on Deadlock** | Net $F_x = 0$ against perpendicular wall | **Canal Streamline Flow**: Natural tangent potential turns robot smoothly along wall surface without stopping. |
| **Pacing / Oscillation** | Bouncing between opposing walls | **Pacing Trap Detector**: Measures net displacement over 2.5s (<0.35m) and locks a reverse bypass direction for 4.0s. |
| **U-Trap Dead Ends** | Stuck in concave pocket | **Boredom Engine**: If robot stalls for $> 2.5\text{s}$, initiates a forced escape turn toward the open side. |
| **Corridor Misdirection** | Exploring dead-ends repeatedly | **Corridor Exploration Memory**: Discretized grid tracking visit frequency strictly within local corridor ($dx < 0.35\text{m}$). |

---

### No-Encoder Autonomous Navigation: Why It Works

The robot's 3 DC motors do not have wheel encoders. The system navigates robustly through closed-loop **reactive sensor feedback**:
1. **Sensors Over Wheels**: The APF loop operates at 10 Hz directly on real-time distances from the 3 ultrasonic sensors and camera floor horizon. If a wall is 20 cm ahead, it steers away until open space is measured, regardless of wheel slip.
2. **Autonomous Exploration Default**: When launched without an external target, `NavigatorNode` initializes an exploration vector $(x=4.0\text{m}, y=0.0\text{m})$ and dead-reckoning odometry integration.
3. **Dynamic Target Injection**: Targets can be injected live anytime via ROS 2 CLI:
   ```bash
   ros2 topic pub --once /target_location geometry_msgs/msg/Point "{x: 3.5, y: 1.5, z: 0.0}"
   ```
4. **Visual Servoing**: When a victim is sighted by the camera, the navigator switches to `VICTIM_PURSUIT`, locking onto the victim's bearing and distance directly.

---

## 3. Motor Kinematics (3-Wheel Omni-Directional Drive)

The mobile chassis utilizes a 3-wheel Kiwi omni-directional drive with $120^\circ$ motor spacing. 

[motor_node.py](file:///c:/Users/Huitae/Cal_Hacks/motor_node.py) translates `/cmd_vel` ($v_x, v_y, w$) into motor PWM:

$$f_1 = 0.578 \cdot a_x - 0.333 \cdot a_y + 0.333 \cdot w \cdot \text{MAX\_POWER}$$
$$f_2 = 0.567 \cdot a_y + 0.333 \cdot w \cdot \text{MAX\_POWER}$$
$$f_3 = -0.578 \cdot a_x - 0.333 \cdot a_y + 0.333 \cdot w \cdot \text{MAX\_POWER}$$

* **Motor A**: Driven by $f_1$ (Pins 17, 27)
* **Motor B**: Driven by $f_3$ (Pins 22, 23)
* **Motor C**: Driven by $f_2$ (Pins 24, 25)

### Hardware Protection Features:
* **Deadband Static-Friction Compensation**: Any speed below motor static friction is boosted to `MIN_POWER_CUTOFF = 12` so wheels start rolling reliably.
* **Pure Rotation in Place**: Pure yaw turns ($v_x=0, v_y=0, w \neq 0$) spin all 3 wheels synchronously without zero-multiplication clipping.
* **Safety Watchdog**: If Wi-Fi communication drops for $> 0.35\text{s}$, power to all 3 motors is automatically cut.

---

## 4. Computer Vision & Sensor Perception

Detailed perception documentation is available in **[docs/VISION_README.md](docs/VISION_README.md)**.

### Low-Latency JPEG CompressedImage Streaming
* **Problem**: Uncompressed `sensor_msgs/Image` (230–614 KB) saturated Wi-Fi UDP buffers, dropping frame rates to 0.02 Hz.
* **Solution**: Captured frames are compressed to JPEG quality 60 (~7 KB/frame) and published over `qos_profile_sensor_data` to `/image_raw/compressed`. Video latency dropped to **<15ms at solid 20 FPS**.

### Floor Horizon Canny Edge Segmentation
* Crops the lower 50% floor region of `/image_raw/compressed`.
* Partitions into **Left (0–33%)**, **Center (33–66%)**, and **Right (66–100%)** sectors.
* Measures edge pixel density to estimate obstacle proximity ($0.15\text{m}$ to $2.00\text{m}$).

### 100% Offline Human Victim Detection
* Primary: YOLOv8-Nano (`yolov8n.pt`) offline PyTorch inference.
* Fallback: Built-in OpenCV Histogram of Oriented Gradients (HOG) People Detector.
* Emits relative victim coordinates to `/detected_person` (`geometry_msgs/msg/Point`).

### Sensor Fusion Envelope
The base station continuously blends vision distances and ultrasonic sonar ranges at 10 Hz:
$$\text{Clearance}_{\text{sector}} = \min(\text{Dist}_{\text{vision}},\; \text{Dist}_{\text{ultrasonic}})$$
Emits unified clearances to `/obstacle_status` (`geometry_msgs/msg/Vector3`).

---

## 5. Docker & FastDDS Shared Memory Architecture

When running multiple containers on ASUS GX10, ROS 2 FastDDS default Shared Memory (`/dev/shm`) transport isolates containers from each other, resulting in topics discovering publishers but receiving 0 bytes.

### The Production Solution
1. **Container Runner (`run_gx10.sh`)**:
   Launches the unified container with `--net=host --ipc=host`, allowing all processes to share host network and IPC namespace:
   ```bash
   sudo docker run -it \
       --name robot_base \
       --net=host \
       --ipc=host \
       -v /home/asus/calhacks:/workspace \
       -w /workspace \
       ros:humble-robot
   ```
2. **FastDDS UDPv4 Configuration (`fastdds_no_shm.xml`)**:
   Forces pure UDPv4 multicast communication across LAN and containers:
   ```bash
   export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/fastdds_no_shm.xml
   ```
3. **Single-Process Unified Base Station (`launch_gx10.py`)**:
   Perception (`VisualWallDetector`) and Navigation (`NavigatorNode`) run in a single process using `MultiThreadedExecutor(num_threads=4)` for zero IPC overhead.

---

## 6. Offline LAN Synchronization

Because hackathon routers operate without public internet, code is pushed directly from Windows via batched SCP:

* **Sync Raspberry Pi only**:
  ```powershell
  cd c:\Users\Huitae\Cal_Hacks
  .\sync_pi.ps1
  ```
* **Sync ASUS GX10 Base Station only**:
  ```powershell
  cd c:\Users\Huitae\Cal_Hacks
  .\sync_gx10.ps1
  ```
* **Sync All Machines**:
  ```powershell
  cd c:\Users\Huitae\Cal_Hacks
  .\sync_all.ps1
  ```

---

## 7. Step-by-Step Production Bringup Guide

### Step 1: Deploy Latest Code from Windows
```powershell
cd c:\Users\Huitae\Cal_Hacks
.\sync_all.ps1
```

### Step 2: Launch Hardware on Raspberry Pi (192.168.0.10)
In Pi Terminal 1 (Sensors & Camera):
```bash
ssh robot1@192.168.0.10
python3 ~/pi_bringup.py
```
*(Verifies camera opened @ 20 FPS and 3x ultrasonic sensors initialized).*

In Pi Terminal 2 (Motors):
```bash
ssh robot1@192.168.0.10
python3 ~/motor_node.py
```
*(Verifies motor driver listening on `/cmd_vel`).*

### Step 3: Launch AI Base Station on ASUS GX10 (192.168.0.232)
```bash
ssh asus@192.168.0.232
chmod +x /home/asus/calhacks/run_gx10.sh
/home/asus/calhacks/run_gx10.sh
```
Inside the container:
```bash
python3 launch_gx10.py
```

### Step 4: Verify Live System in a Second GX10 Terminal
Open a new GX10 terminal and run `/home/asus/calhacks/run_gx10.sh`:
```bash
# Verify obstacle clearances:
ros2 topic echo /obstacle_status

# Verify motor drive commands:
ros2 topic echo /cmd_vel

# Inject a target coordinate anytime:
ros2 topic pub --once /target_location geometry_msgs/msg/Point "{x: 3.5, y: 1.0, z: 0.0}"
```

---

## 8. Simulation Testing & Visual Scenarios

For rapid desktop algorithm testing without robot hardware, run the built-in terminal simulator:

```bash
# Launch APF Navigator:
ros2 run robot_core navigator_node

# In another terminal, launch physics simulator with map:
ros2 run robot_core mock_robot_sim --ros-args -p scenario:=office
```

### Available Scenarios:
* **`office`**: Building layout with hallway entrance and interior room doorways.
* **`multilayer`**: S-Curve alternating slalom testing sequential wall transitions.
* **`triple`**: 3 alternating maze walls (Up -> Down -> Up).
* **`u_trap`**: Concave pocket testing the Boredom Engine escape pivot.
* **`clutter`**: Multiple standalone pillars and rubble columns.

### Real-Time Terminal ASCII Visualizer:
```text
========================================
 🤖 MOCK ROBOT APF SIMULATOR (OFFICE)
========================================
+----------------------------------+
|           #         #            |
|           #         #            |
|           #         #            |
|           #         #        T   |
|   >       #                      |
|           #                      |
|                                  |
+----------------------------------+
 Robot Pose:  X:  0.85m, Y:  1.12m | Heading:  18°
 Goal Dist:    3.27m  | Target: (4.0, 2.0)
 Clearances:  Front: 0.85m | Left: 1.88m | Right: 0.96m
 Motors Cmd:  linear.x =  0.25 m/s | angular.z =  0.12 rad/s
========================================
```

---

## 9. ROS 2 Topic Interfaces

| Topic | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | Pi $\rightarrow$ GX10 | Hardware-encoded JPEG camera stream @ 20 FPS (<15ms latency). |
| `/ultrasonic_center` | `std_msgs/Float32` | Pi $\rightarrow$ GX10 | Center HC-SR04 distance in cm (GPIO 12/13). |
| `/ultrasonic_left` | `std_msgs/Float32` | Pi $\rightarrow$ GX10 | Left HC-SR04 distance in cm (GPIO 20/21). |
| `/ultrasonic_right` | `std_msgs/Float32` | Pi $\rightarrow$ GX10 | Right HC-SR04 distance in cm (GPIO 5/6). |
| `/obstacle_status` | `geometry_msgs/Vector3` | GX10 Internal | Fused clearances ($x$=front, $y$=left, $z$=right in meters). |
| `/detected_person` | `geometry_msgs/Point` | Vision $\rightarrow$ Nav | Relative victim location ($x$=distance, $y$=bearing deg, $z$=confidence). |
| `/target_location` | `geometry_msgs/Point` | User / RF $\rightarrow$ Nav | Destination coordinates ($x, y, z$). |
| `/cmd_vel` | `geometry_msgs/Twist` | GX10 $\rightarrow$ Pi | Linear speed ($v_x, v_y$ in m/s) and angular steering rate ($w$ in rad/s). |

---

## 10. Repository File Tree

```text
Cal_Hacks/
├── README.md                                  # Complete system architecture and operation guide
├── docs/
│   └── VISION_README.md                       # Dedicated perception, streaming, and fusion guide
├── sync_all.ps1                               # One-click LAN sync to both GX10 and Pi
├── sync_pi.ps1                                # Quick sync to Raspberry Pi
├── sync_gx10.ps1                              # Quick sync to ASUS GX10
├── run_gx10.sh                                # Robust Docker runner on GX10 host (--net=host --ipc=host)
├── launch_gx10.py                             # Single-process AI Base Station (Perception + Navigation)
├── pi_bringup.py                              # Raspberry Pi hardware bringup (Camera + 3x Ultrasonic)
├── motor_node.py                              # ROS 2 3-wheel omni motor driver with safety watchdog
├── fastdds_no_shm.xml                         # FastDDS UDPv4 transport profile bypassing SHM isolation
├── yolov8n.pt                                 # Offline YOLOv8-Nano weights (6.2 MB)
└── src/
    ├── robot_core/                            # ROS 2 Navigation Package
    │   ├── package.xml
    │   ├── setup.py
    │   └── robot_core/
    │       ├── navigator_node.py              # Streamline APF Engine & Autonomous Exploration
    │       ├── mock_robot_sim.py              # Physics simulator, CCD, raycaster & ASCII visualizer
    │       └── vision_obstacle_node.py        # Perception fusion node
    └── robot_vision/                          # ROS 2 Perception Package
        ├── package.xml
        ├── setup.py
        └── robot_vision/
            ├── visual_wall_detector.py        # Floor Canny edge detector & ultrasonic fusion
            ├── detector.py                    # Offline YOLOv8 and OpenCV HOG victim detector
            ├── pi_cam_node.py                 # Logitech Brio JPEG streaming node
            ├── ultrasonic_node.py             # Triple HC-SR04 ROS 2 publisher
            └── test_camera.py                 # Standalone camera HUD tester
```
