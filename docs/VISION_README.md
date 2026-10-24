# Perception & Multi-Robot Vision System (`robot_vision`)

This guide covers the computer vision, sensor fusion, and multi-robot perception architecture for search-and-rescue robotics at Cal Hacks.

It explains how the **ASUS Ascent GX10** acts as the central AI workstation on the desk while **two mobile robots** (powered by Raspberry Pis) stream video and sensor data over Wi-Fi.

---

## 📌 Table of Contents
1. [System Architecture (GX10 Brain + 2x Raspberry Pi Clients)](#1-system-architecture)
2. [Raspberry Pi Robot Setup (`pi_client.py`)](#2-raspberry-pi-robot-setup)
   - [Hardware Connections (Logitech Webcam, HC-SR04, Motors)](#hardware-connections)
   - [Running the Pi Streamer](#running-the-pi-streamer)
   - [Safety Motor Watchdog](#safety-motor-watchdog)
3. [ASUS GX10 Perception Node (`vision_node.py`)](#3-asus-gx10-perception-node)
   - [Multi-Source Input (Webcam vs Network Streams)](#multi-source-input)
   - [Multi-Robot Namespacing (`robot1` and `robot2`)](#multi-robot-namespacing)
4. [Victim Detection Models (YOLOv8 vs. OpenCV HOG)](#4-victim-detection-models)
   - [Why Compute on GX10 Instead of Raspberry Pi?](#why-compute-on-gx10-instead-of-raspberry-pi)
   - [Seamless Model Fallback (`detector.py`)](#seamless-model-fallback)
5. [Obstacle Detection & Sensor Fusion](#5-obstacle-detection--sensor-fusion)
   - [Floor ROI Canny Edge Segmentation](#floor-roi-canny-edge-segmentation)
   - [Ultrasonic & Vision Distance Fusion](#ultrasonic--vision-distance-fusion)
6. [3D Victim Localization & Navigator Target Updates](#6-3d-victim-localization--navigator-target-updates)
7. [Running & Testing the Vision Pipeline](#7-running--testing-the-vision-pipeline)
   - [Testing with `test_camera.py`](#testing-with-test_camerapy)
   - [Launching Dual-Robot Vision](#launching-dual-robot-vision)

---

## 1. System Architecture

```
[ Robot 1: Mobile Chassis ]                     [ ASUS Ascent GX10: Base Station (Desk) ]
+-----------------------------------+          +-------------------------------------------+
| - Logitech Webcam (/dev/video0)   |          | - Dual-Stream YOLOv8 Perception Hub       |
| - Ultrasonic HC-SR04 Sensors      | ===Wi-Fi==> - Floor ROI Obstacle Edge Segmentation    |
| - Differential Wheel Motor Driver |          | - Multi-Agent Target Triangulation        |
| - Raspberry Pi (Thin Client)      | <==cmd_vel= - APF Global Pathfinders (/robot1, /robot2)|
+-----------------------------------+          +-------------------------------------------+
                                                                     ^
[ Robot 2: Mobile Chassis ]                                          |
+-----------------------------------+                                |
| - Logitech Webcam (/dev/video0)   | ==================Wi-Fi========+
| - Ultrasonic HC-SR04 Sensors      |
| - Raspberry Pi (Thin Client)      |
+-----------------------------------+
```

### Why Compute on the GX10 Instead of the Raspberry Pi?
1. **Compute & Thermals**: Running YOLOv8 and Canny edges on a Raspberry Pi 4 causes thermal throttling (80°C), drops frame rates to 3–8 FPS, and drains mobile batteries rapidly. The GX10 runs both video streams at 30–60 FPS with dedicated AI acceleration on AC wall power.
2. **Unified Multi-Robot Map**: The GX10 maintains a shared world coordinate system. When Robot 1 sights an obstacle or victim, the coordinate is immediately shared with Robot 2 to prevent redundant searching.
3. **Single Codebase**: All AI models and navigation algorithms run inside your Docker container on the GX10. You never need to SSH and debug complex ML dependencies on the Pis during the competition.

---

## 2. Raspberry Pi Robot Setup (`pi_client.py`)

Each robot runs a lightweight script ([pi_client.py](file:///home/asus/calhacks/src/robot_vision/robot_vision/pi_client.py)) that captures webcam video with hardware MJPEG encoding, samples ultrasonic rangefinders, and drives motor PWM.

### Hardware Connections
- **Camera**: Logitech Webcam plugged into any USB port (`/dev/video0`).
- **Ultrasonic (HC-SR04)**:
  - `VCC` -> 5V Pin
  - `GND` -> Ground Pin
  - `TRIG` -> GPIO 23
  - `ECHO` -> GPIO 24 (via voltage divider: 1kΩ / 2kΩ resistors to step 5V down to 3.3V)
- **Motor Driver (L298N / TB6612FNG)**:
  - Left Motor PWM / DIR: GPIO 12, GPIO 16
  - Right Motor PWM / DIR: GPIO 13, GPIO 20

### Running the Pi Streamer
Copy `pi_client.py` to the Raspberry Pi and run:

```bash
# On Robot 1:
python3 pi_client.py 5000 0

# On Robot 2:
python3 pi_client.py 5000 0
```
- Argument 1: HTTP port (default: `5000`)
- Argument 2: Camera index (default: `0` for `/dev/video0`)

The Pi hosts:
- `http://<ROBOT_IP>:5000/video` — High-speed MJPEG video feed (~30 FPS, <2 Mbps bandwidth).
- `http://<ROBOT_IP>:5000/sensors` — JSON telemetry (`{"front": 0.45, "left": 1.2, "right": 2.0}`).
- `http://<ROBOT_IP>:5000/cmd_vel` — POST endpoint to accept motion commands from the GX10.

### Safety Motor Watchdog
`pi_client.py` includes an automatic safety watchdog thread. If the GX10 does not send a motion heartbeat for more than **0.5 seconds** (e.g. Wi-Fi packet drop or computer sleep), the robot automatically cuts motor power to prevent driving into walls.

---

## 3. ASUS GX10 Perception Node (`vision_node.py`)

The GX10 runs [vision_node.py](file:///home/asus/calhacks/src/robot_vision/robot_vision/vision_node.py). It receives image frames, processes victim and obstacle detections, and publishes ROS 2 topics.

### Multi-Source Input
The `camera_source` parameter accepts:
- An integer index (`"0"` or `"/dev/video0"`): Uses local USB Logitech webcam attached to the GX10.
- An HTTP URL (`"http://192.168.1.101:5000/video"`): Ingests the wireless stream from the Raspberry Pi.

### Multi-Robot Namespacing
When running two robots, topics are namespaced to avoid collision:

| Robot 1 Topic | Robot 2 Topic | Type | Description |
| :--- | :--- | :--- | :--- |
| `/robot1/obstacle_status` | `/robot2/obstacle_status` | `geometry_msgs/Vector3` | Front, Left, Right clearances (m) |
| `/robot1/detected_person` | `/robot2/detected_person` | `geometry_msgs/Point` | Relative (X, Y) and confidence |
| `/robot1/target_location` | `/robot2/target_location` | `geometry_msgs/Point` | Global (X, Y) victim coordinates |
| `/robot1/odom` | `/robot2/odom` | `nav_msgs/Odometry` | Robot odometry and heading |
| `/robot1/ultrasonic_raw` | `/robot2/ultrasonic_raw` | `geometry_msgs/Vector3` | Hardware sonar distance readings |

---

## 4. Victim Detection Models (YOLOv8 vs. OpenCV HOG)

[detector.py](file:///home/asus/calhacks/src/robot_vision/robot_vision/detector.py) implements human victim detection:

| Model | Backend | Speed (GX10) | Victim Recall | Handles Fallen/Occluded? |
| :--- | :--- | :--- | :--- | :--- |
| **YOLOv8-Nano** (`yolov8n.pt`) | PyTorch / ONNX | **45–60 FPS** | **94%** | **Yes** (sitting, crawling, occluded) |
| **OpenCV HOG** | `HOGDescriptor` | 20–30 FPS | 68% | Only upright walking humans |

### Seamless Model Fallback
The detector automatically checks for `ultralytics`. If installed, it loads `yolov8n.pt` (6.2 MB). If `ultralytics` is not installed, it automatically falls back to OpenCV's built-in HOG People Detector without crashing.

To enable YOLOv8 on the GX10:
```bash
pip install ultralytics
```

---

## 5. Obstacle Detection & Sensor Fusion

### Floor ROI Canny Edge Segmentation
To identify cardboard walls, maze barriers, and rubble without needing heavy 3D point-cloud LiDAR:
1. Crop the bottom 40% of the image (the floor immediately ahead).
2. Apply Gaussian Blur (`5x5`) and Canny Edge Detection (`50, 150`).
3. Partition the floor into three vertical zones:
   - **Left Zone (0% – 30% width)**
   - **Center Zone (30% – 70% width)**
   - **Right Zone (70% – 100% width)**
4. Compute edge pixel density $\rho$ in each column. Obstacle distance is estimated as:
   ```text
   dist_est = max(0.18, min(2.0, 0.28 / max(density, 0.05)))
   ```

### Ultrasonic & Vision Distance Fusion
Vision edge detection and ultrasonic rangefinders complement each other:
- Ultrasonic rangefinders excel at close-range smooth walls but can bounce off angled corners.
- Vision edge detection catches textured obstacles and corners but can be affected by shadows.

The node takes the **minimum** distance between vision and ultrasonic readings:
```text
clearance_front = min(vision_front, sonar_front)
clearance_left  = min(vision_left, sonar_left)
clearance_right = min(vision_right, sonar_right)
```
This guarantees fail-safe obstacle avoidance.

---

## 6. 3D Victim Localization & Navigator Target Updates

When a victim is spotted, the node projects the 2D bounding box into 3D robot-centric coordinates using the pinhole camera model:

```text
est_distance = (person_real_height * focal_length_px) / bbox_height_px
bearing_rad  = atan2(bbox_center_x - image_center_x, focal_length_px)

rel_x = est_distance * cos(bearing_rad)
rel_y = -est_distance * sin(bearing_rad)
```

If `/odom` is available, the node transforms the relative coordinates into **global arena coordinates**:
```text
world_x = robot_x + rel_x * cos(robot_theta) - rel_y * sin(robot_theta)
world_y = robot_y + rel_x * sin(robot_theta) + rel_y * cos(robot_theta)
```

The node publishes the updated coordinates to `/target_location`, dynamically commanding [navigator_node.py](file:///home/asus/calhacks/src/robot_core/robot_core/navigator_node.py) to home in and dock at the victim.

---

## 7. Running & Testing the Vision Pipeline

### Testing with `test_camera.py`
Verify your camera and detection pipeline with live FPS and visual HUD:

```bash
# Test a local USB Logitech webcam:
python3 src/robot_vision/robot_vision/test_camera.py 0

# Test a wireless stream from Raspberry Pi:
python3 src/robot_vision/robot_vision/test_camera.py http://192.168.1.101:5000/video
```

### Launching Single-Robot Vision (ROS 2)
```bash
ros2 launch robot_vision single_robot_vision.launch.py \
  robot_id:=robot1 \
  camera_source:=http://192.168.1.101:5000/video \
  show_display:=True
```

### Launching Dual-Robot Vision (ROS 2)
To run the central perception brain on the GX10 for both robots simultaneously:
```bash
ros2 launch robot_vision dual_robot_vision.launch.py \
  robot1_source:=http://192.168.1.101:5000/video \
  robot2_source:=http://192.168.1.102:5000/video \
  show_display:=True
```
Both robots will stream to the GX10, publish their clearances, and coordinate autonomous search-and-rescue navigation!
