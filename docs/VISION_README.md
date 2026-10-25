# Perception & Real-Time Vision System (`robot_vision`)

This guide covers the computer vision, sensor fusion, network optimization, and perception architecture for autonomous search-and-rescue robotics developed for Cal Hacks.

It explains how the **ASUS Ascent GX10 AI Workstation** acts as the central compute base station on the desk while the mobile robot (powered by a **Raspberry Pi 4**) streams low-latency sensor and video telemetry over local Wi-Fi.

---

## 📌 Table of Contents
1. [System Architecture (ASUS GX10 Brain + Raspberry Pi Robot Host)](#1-system-architecture)
2. [Wi-Fi Network Optimization: Raw Image vs. JPEG Compressed Streaming](#2-wi-fi-network-optimization)
3. [Raspberry Pi Robot Hardware & Sensors](#3-raspberry-pi-robot-hardware--sensors)
   - [Hardware Pinout (Motors & Ultrasonics)](#hardware-pinout)
   - [Pi Bringup (`pi_bringup.py`)](#pi-bringup)
   - [Omni-Directional Motor Driver (`motor_node.py`)](#omni-directional-motor-driver)
4. [ASUS GX10 Perception Pipeline (`visual_wall_detector.py`)](#4-asus-gx10-perception-pipeline)
   - [Floor ROI Canny Edge Horizon Scan](#floor-roi-canny-edge-horizon-scan)
   - [Ultrasonic & Vision Sensor Fusion](#ultrasonic--vision-sensor-fusion)
   - [Victim Detection: YOLOv8 vs. OpenCV HOG](#victim-detection-yolov8-vs-opencv-hog)
5. [Single-Process Base Station Integration (`launch_gx10.py`)](#5-single-process-base-station-integration)
6. [Offline LAN Synchronization (`sync_all.ps1`, `sync_pi.ps1`, `sync_gx10.ps1`)](#6-offline-lan-synchronization)
7. [Running & Testing the Vision Pipeline](#7-running--testing-the-vision-pipeline)

---

## 1. System Architecture

```text
[ Mobile Robot: Raspberry Pi 4 (192.168.0.10) ]              [ ASUS Ascent GX10 AI Base Station (192.168.0.232) ]
+-------------------------------------------------+          +------------------------------------------------------+
| - Logitech Brio USB Camera (/dev/video0)        |          | - Single-Process Unified Executor (launch_gx10.py)   |
|   Streams /image_raw/compressed (JPEG @ 20 FPS) | ==Wi-Fi==> - VisualWallDetector:                                 |
| - Triple HC-SR04 Rangefinders:                  |          |     * Floor Horizon Canny Edge Segmentation          |
|   /ultrasonic_center (Trig 12, Echo 13)         |          |     * 100% Offline Person Detection (YOLOv8 / HOG)   |
|   /ultrasonic_left   (Trig 20, Echo 21)         |          |     * 10Hz Continuous Sensor Fusion -> /obstacle_status|
|   /ultrasonic_right  (Trig 5,  Echo 6)          |          | - NavigatorNode:                                     |
| - 3-Wheel Omni DC Motors (motor_node.py)        | <==cmd_vel=    * Streamline APF Pathfinding + Dead-End Escape   |
|   L298N H-Bridge PWM (A:17/27, B:22/23, C:24/25)| (Twist)  |     * Autonomous Exploration Mode -> /cmd_vel        |
+-------------------------------------------------+          +------------------------------------------------------+
```

### Why Compute on the GX10 Instead of the Raspberry Pi?
1. **Compute & Thermals**: Running YOLOv8 and multi-sector Canny edge detection on a Raspberry Pi 4 pushes CPU temperatures to 80°C, causes severe thermal throttling, drops video to 3–6 FPS, and drains robot batteries rapidly. The GX10 runs computer vision at full 30–60 FPS with dedicated GPU acceleration on AC wall power.
2. **Microsecond Internal Latency**: By running both perception (`VisualWallDetector`) and path planning (`NavigatorNode`) inside a single multithreaded process on the GX10, obstacle clearances propagate directly into motion commands with sub-millisecond latency.
3. **100% Offline Capability**: All AI models (YOLOv8-Nano `yolov8n.pt` and OpenCV HOG People Detector) execute completely offline without requiring an active internet connection.

---

## 2. Wi-Fi Network Optimization: Raw vs. Compressed

In early development, transmitting uncompressed ROS 2 images (`sensor_msgs/Image`) caused severe network choke over standard Wi-Fi routers:

| Metric | Raw `sensor_msgs/Image` | JPEG `sensor_msgs/CompressedImage` |
| :--- | :--- | :--- |
| **Payload Size** | 230 KB (320x240 RGB) to 614 KB (640x480 YUYV) | **~7 KB per frame** (JPEG Quality 60) |
| **UDP Packets / Frame** | ~160 to 420 UDP fragments | **~5 UDP packets** |
| **Packet Loss Result** | Any single lost fragment drops the frame | Negligible packet loss risk |
| **Effective Frame Rate** | **0.019 Hz to 0.14 Hz** (Severe Dropouts) | **Solid 20.0 FPS** (Zero Drops) |
| **DDS Transport Profile** | Reliable TCP (High latency on reconnect) | Best-effort `qos_profile_sensor_data` (<15ms latency) |

**Solution implemented in `pi_bringup.py` / `pi_cam_node.py`**:
Frames captured from the Logitech Brio are compressed to JPEG in memory on the Raspberry Pi using OpenCV hardware-accelerated SIMD instructions and published as `sensor_msgs/CompressedImage` to `/image_raw/compressed`.

---

## 3. Raspberry Pi Robot Hardware & Sensors

### Hardware Pinout

The Raspberry Pi 40-pin header is wired to avoid all GPIO conflicts between the 3 DC motors and the 3 ultrasonic sensors:

| Subsystem | Component | Function | BCM GPIO | Header Physical Pin |
| :--- | :--- | :--- | :--- | :--- |
| **Motors** | **Motor A** | Forward / Reverse PWM | **GPIO 17, 27** | Pins 11, 13 |
| | **Motor B** | Forward / Reverse PWM | **GPIO 22, 23** | Pins 15, 16 |
| | **Motor C** | Forward / Reverse PWM | **GPIO 24, 25** | Pins 18, 22 |
| **Ultrasonics** | **Center** | Trigger / Echo | **GPIO 12, 13** | Pins 32, 33 |
| | **Left** | Trigger / Echo | **GPIO 20, 21** | Pins 38, 40 |
| | **Right** | Trigger / Echo | **GPIO 5, 6** | Pins 29, 31 |
| **Power** | **Sensors** | VCC (5V) & Ground | **5V & GND** | Pins 2, 4 & Pins 30, 34, 39 |

### Pi Bringup (`pi_bringup.py`)
Single-process multithreaded hardware node on the Raspberry Pi running:
1. `UnifiedPiCamera`: Streams Logitech Brio JPEG compressed video (`/image_raw/compressed`) at 20 FPS.
2. `UnifiedUltrasonic`: Reads Center, Left, and Right HC-SR04 sensors concurrently and publishes distances in cm at 5 Hz to `/ultrasonic_center`, `/ultrasonic_left`, and `/ultrasonic_right`.
3. **Fault-Tolerant Initialization**: Automatically unexports any stale sysfs pins on startup and safely frees lingering `lgpio` kernel handles.

### Omni-Directional Motor Driver (`motor_node.py`)
Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`) and translates linear velocity ($v_x, v_y$) and yaw rate ($w$) into 3-wheel Kiwi omni-directional drive PWM signals:
- **Kinematic Transformation**:
  $$f_1 = 0.578 \cdot a_x - 0.333 \cdot a_y + 0.333 \cdot w \cdot \text{MAX\_POWER}$$
  $$f_2 = 0.567 \cdot a_y + 0.333 \cdot w \cdot \text{MAX\_POWER}$$
  $$f_3 = -0.578 \cdot a_x - 0.333 \cdot a_y + 0.333 \cdot w \cdot \text{MAX\_POWER}$$
- **Deadband Static-Friction Boost**: Low speeds below motor static friction are boosted to `MIN_POWER_CUTOFF = 12` so wheels move reliably.
- **Safety Watchdog**: If Wi-Fi communication drops for $> 0.35\text{s}$, the node cuts power to all motors immediately.

---

## 4. ASUS GX10 Perception Pipeline (`visual_wall_detector.py`)

### Floor ROI Canny Edge Horizon Scan
To detect obstacles, rubble, and maze walls without 3D LiDAR:
1. Crops the lower 50% of the camera image (the floor plane directly ahead).
2. Applies bilateral filtering and Canny edge detection.
3. Segments the floor plane into three directional sectors:
   - **Left Sector** (0% to 33% width)
   - **Center Sector** (33% to 66% width)
   - **Right Sector** (66% to 100% width)
4. Calculates edge pixel density in each sector to estimate distance ($0.15\text{m}$ to $2.00\text{m}$).

### Ultrasonic & Vision Sensor Fusion
Ultrasonics excel at measuring flat drywall up close, while vision edge detection catches textured obstacles and corners:
- The node samples `/ultrasonic_center`, `/ultrasonic_left`, and `/ultrasonic_right`.
- Merges readings using a **minimum-clearance safety envelope**:
  $$\text{Clearance}_{\text{sector}} = \min(\text{Dist}_{\text{vision}},\; \text{Dist}_{\text{ultrasonic}})$$
- Emits continuous fused clearances to `/obstacle_status` (`geometry_msgs/msg/Vector3`) at 10 Hz.

### Victim Detection: YOLOv8 vs. OpenCV HOG
The detection engine runs in [detector.py](file:///c:/Users/Huitae/Cal_Hacks/src/robot_vision/robot_vision/detector.py):
- **Primary**: YOLOv8-Nano (`yolov8n.pt`) with full offline PyTorch inference.
- **Fallback**: Built-in OpenCV Histogram of Oriented Gradients (HOG) People Detector (100% offline with zero extra dependencies).
- When a human victim is detected, publishes relative target coordinates to `/detected_person` (`geometry_msgs/msg/Point`).

---

## 5. Single-Process Base Station Integration (`launch_gx10.py`)

Running perception and navigation across separate Docker containers often leads to DDS Shared Memory (`/dev/shm`) isolation where nodes discover each other but drop payload packets.

[launch_gx10.py](file:///c:/Users/Huitae/Cal_Hacks/launch_gx10.py) solves this by running both:
1. `VisualWallDetector` (Perception)
2. `NavigatorNode` (APF Brain & Autonomous Exploration)

inside a **single process** using a `MultiThreadedExecutor(num_threads=4)`. This guarantees microsecond response latency and zero DDS packet loss.

---

## 6. Offline LAN Synchronization

Because hackathon venues often provide local routers without public internet access, code deployment uses LAN `scp` batching:

* **Windows to Raspberry Pi**:
  ```powershell
  cd c:\Users\Huitae\Cal_Hacks
  .\sync_pi.ps1
  ```
* **Windows to ASUS GX10**:
  ```powershell
  cd c:\Users\Huitae\Cal_Hacks
  .\sync_gx10.ps1
  ```
* **Windows to Both Simultaneously**:
  ```powershell
  cd c:\Users\Huitae\Cal_Hacks
  .\sync_all.ps1
  ```

---

## 7. Running & Testing the Vision Pipeline

### Step 1: Launch Robot Sensors & Motors on Raspberry Pi
```bash
ssh robot1@192.168.0.10
# Terminal 1: Sensors
python3 ~/pi_bringup.py

# Terminal 2: Motors
python3 ~/motor_node.py
```

### Step 2: Launch AI Base Station on ASUS GX10
```bash
ssh asus@192.168.0.232
chmod +x /home/asus/calhacks/run_gx10.sh
/home/asus/calhacks/run_gx10.sh
```
Inside the container:
```bash
python3 launch_gx10.py
```

### Step 3: Monitor Live Telemetry
In a second GX10 terminal (`/home/asus/calhacks/run_gx10.sh`):
```bash
# Monitor fused obstacle clearances:
ros2 topic echo /obstacle_status

# Monitor navigation motor commands:
ros2 topic echo /cmd_vel
```
Place an obstacle or hand in front of the robot: observe clearances drop in `/obstacle_status` and the navigator issue avoidance steering commands in `/cmd_vel`!
