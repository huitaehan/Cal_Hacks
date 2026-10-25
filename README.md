# 🚨 Autonomous Multi-Agent Search & Rescue System
### CalHacks 13.0 | RF Through-Wall Sensing • Edge AI Robotics • Real-Time Voice Intelligence

An end-to-end autonomous search-and-rescue disaster response ecosystem engineered for extreme environments (collapsed buildings, smoke-filled corridors, and disaster zones) where optical cameras fail and GPS is unavailable.

The system combines **camera-free through-wall Wi-Fi Channel State Information (CSI) human sensing**, a **reactive 3-wheel omni-directional rescue robot with edge AI perception and APF navigation**, and a **low-latency voice AI dispatch agent** for continuous victim triage.

---

## 🏗️ System Architecture & Data Flow

```text
       +---------------------------------------------------------------------------------+
       |                     PILLAR 1: THROUGH-WALL RF PERCEPTION                         |
       |  4x ESP32-C5 Nodes (ESP-NOW 2.4 GHz)  ====>  Gateway Node (COM USB)             |
       |  Performs Wi-Fi CSI Doppler disturbance monitoring through walls               |
       |  presence_server.py (Host) -> Computes Victim (X, Y) Coordinates @ 4 Hz        |
       +---------------------------------------+-----------------------------------------+
                                               |
                                     Target (X, Y) /api/state
                                               v
+------------------------------------------------------------------------------------------------+
|                   PILLAR 2: AUTONOMOUS SEARCH & RESCUE ROBOT PLATFORM                          |
|                                                                                                |
|   [ Mobile Rover: Raspberry Pi 4 ]                      [ ASUS Ascent GX10 AI Base Station ]  |
|   +--------------------------------------+              +-----------------------------------+  |
|   |             THE EYES                 |              |             THE BRAIN             |  |
|   | - Logitech Brio USB Camera           | ==Wi-Fi===>  |  launch_gx10.py                   |  |
|   |   -> /image_raw/compressed (20 FPS)  |              |  - VisualWallDetector (Canny+YOLO)|  |
|   | - 3x HC-SR04 Ultrasonic Sonar        |              |  - Sensor Fusion -> 50cm Buffer   |  |
|   |   -> Left, Center, Right (10 Hz)     |              |  - APF Streamline Navigator Node  |  |
|   +--------------------------------------+              |    * Pure in-place yaw rotations  |  |
|                      ^                                  |    * Straight-line crawl (0.015m) |  |
|                      | /cmd_vel (Twist)                 +-----------------+-----------------+  |
|   +------------------+-------------------+                                |                    |
|   |             THE BODY                 |                                |                    |
|   | - motor_node.py (3-Wheel Kiwi Omni)  | <==============================+                    |
|   |   A: GPIO 17/27, B: 22/23, C: 24/25  |                                                     |
|   +--------------------------------------+                                                     |
+------------------------------------------------------------------------------------------------+
                                               |
                                    Victim Audio Stream / Contact
                                               v
       +---------------------------------------------------------------------------------+
       |                     PILLAR 3: VOICE AI & FIRST RESPONDER DISPATCH                |
       |  Voice_Cartesia_Mock/ (Cartesia Sonic Streaming + SpacetimeDB State Engine)     |
       |  - Ultra-low latency voice communication with trapped survivor via robot audio  |
       |  - Real-time emergency dispatcher console (dashboard.html) for first responders |
       +---------------------------------------------------------------------------------+
```

---

## 🏛️ The Three Core Pillars

### 1. 📡 Pillar 1: Camera-Free Through-Wall RF Tracking (`wifi detection/`)
* **How It Works**: Four ESP32-C5 RISC-V nodes form a mesh perimeter around the disaster quadrant. Channel State Information (CSI) packets transmitted at 2.4 GHz detect Doppler phase and amplitude disturbances caused by human movement—even through walls.
* **Localization**: The gateway node feeds high-speed CSI measurements into `presence_server.py` and `wander.py`, isolating moving human signatures from static debris and outputting estimated coordinates $(x, y)$.
* **Live Dashboards**: Includes real-time 2D radar visualizer (`static/index.html`) and interactive 3D WebGL digital twin (`static/scene3d.html`).

---

### 2. 🤖 Pillar 2: Autonomous Ground Rescue Robot (`src/` & Root Scripts)
* **Chassis & Kinematics**: 3-wheel Kiwi omni-directional drive with $120^\circ$ separation, running without wheel encoders.
* **Perception Engine**:
  * **Floor Horizon Canny Edge Segmentation**: Real-time obstacle boundary detection from the Logitech Brio camera, streamed as hardware-encoded JPEG (<15ms latency).
  * **Offline YOLOv8-Nano**: Onboard victim visual verification without internet connectivity.
  * **Triple Sonar Array**: Left, Center, and Right HC-SR04 sensors integrated via fast-attack sensor fusion.
* **Navigation Brain (`NavigatorNode`)**:
  * **Streamline Artificial Potential Field (APF)**: Translates RF target pull and obstacle repulsions into smooth navigation paths.
  * **Decoupled Motion**: Pure in-place rotation when steering; strictly dead-straight translation when moving forward (eliminates curvy drifting).
  * **Safety Danger Buffer**: Progressive deceleration starting at $0.80\text{ m}$, bringing the robot to an automatic stop at $0.50\text{ m}$ (50 cm) from walls.
* 📖 *For complete technical details, mathematical formulations, and pinout diagrams, see the [Autonomous Robot System Guide](src/README.md).*

---

### 3. 🎙️ Pillar 3: Interactive Voice AI Dispatcher (`Voice_Cartesia_Mock/`)
* **Voice Agent**: Powered by Cartesia's ultra-low latency text-to-speech and speech-to-text models for human-like conversational triage.
* **SpacetimeDB Sync**: Distributed, real-time reactive database module ensuring synchronized survivor vital statuses across all responder tablets.
* **First Responder HUD**: Live web dashboard (`dashboard.html`) displaying incoming victim audio, transcripts, and status updates.

---

## 📁 Repository Directory Structure

```text
Cal_Hacks/
├── README.md                                  # Unified multi-agent project overview (This file)
├── AGENTS.md                                  # Operational cheat sheet & hardware deployment rules
├── fastdds_no_shm.xml                         # FastDDS UDPv4 transport profile bypassing Docker SHM isolation
├── yolov8n.pt                                 # Offline YOLOv8-Nano neural network weights (6.2 MB)
│
├── src/                                       # [PILLAR 2] Autonomous Robot Navigation & Vision
│   ├── README.md                              # Detailed autonomous robotics & APF system documentation
│   ├── robot_core/                            # ROS 2 APF Navigation & Simulation Package
│   │   ├── robot_core/navigator_node.py       # Core APF Navigation Brain (50cm buffer, in-place yaw)
│   │   └── robot_core/mock_robot_sim.py       # Physics simulator & ASCII visualizer
│   └── robot_vision/                          # ROS 2 Computer Vision & Perception Package
│       ├── robot_vision/visual_wall_detector.py # Floor Canny edge detector & sonar fusion
│       ├── robot_vision/detector.py           # Offline YOLOv8 and OpenCV HOG victim detector
│       ├── robot_vision/pi_cam_node.py        # Logitech Brio JPEG streaming node
│       └── robot_vision/ultrasonic_node.py    # Triple HC-SR04 rangefinder publisher
│
├── wifi detection/                            # [PILLAR 1] Through-Wall Wi-Fi CSI Detection
│   ├── run_dashboard.ps1                      # Start CSI presence server & live dashboard
│   ├── host/presence_server.py                # CSI presence server & API host (port 8000)
│   ├── host/wander.py                         # Link motion detector & signal processing
│   ├── host/static/                           # 2D & 3D Three.js real-time radar visualizers
│   └── firmware/c5_presence_node/             # ESP-IDF firmware for ESP32-C5 sensing nodes
│
├── Voice_Cartesia_Mock/                       # [PILLAR 3] Voice AI & Dispatcher HUD
│   ├── dashboard.html                         # First responder dispatch & survivor status console
│   ├── server.mjs                             # Voice streaming & WebSocket coordination server
│   ├── dispatch.mjs                           # Voice agent call dispatcher
│   └── spacetimedb-module/                    # Real-time survivor status synchronization
│
├── launch_gx10.py                             # Single-process AI Base Station runner (GX10)
├── motor_node.py                              # Raspberry Pi Kiwi omni motor driver (PWM with safety watchdog)
├── pi_bringup.py                              # Raspberry Pi sensor bringup (Camera + 3x Ultrasonic)
├── run_gx10.sh                                # Docker launch script on ASUS GX10 host (--net=host --ipc=host)
├── sync_all.ps1                               # One-click LAN sync to both GX10 and Pi
├── sync_gx10.ps1                              # Fast SCP sync to ASUS GX10 AI base station
└── sync_pi.ps1                                # Fast SCP sync to Raspberry Pi robot
```

---

## 🚀 Unified Quickstart & Bringup Guide

### Step 1: Deploy Wi-Fi Through-Wall Sensing Perimeter
1. Place 4 ESP32-C5 boards at the quadrant corners at waist height. Connect the gateway board (`d0:cf:13:e1:2e:30`) via USB.
2. Launch the sensing server and visualizer:
   ```powershell
   cd "wifi detection"
   .\run_dashboard.ps1 -Record
   ```
3. Open `http://localhost:8000` for 2D radar tracking or `http://localhost:8000/3d` for the 3D scene.

---

### Step 2: Bring Up Mobile Rescue Rover (Raspberry Pi 4)
1. SSH into the robot or launch via terminal (`robot1@192.168.0.10`):
   ```bash
   # Terminal 1 - Start Camera & 3x Ultrasonic Rangefinders:
   python3 ~/pi_bringup.py

   # Terminal 2 - Start Omni Motor Driver:
   python3 ~/motor_node.py
   ```

---

### Step 3: Launch AI Base Station (ASUS Ascent GX10)
1. Sync latest navigation code from developer PC:
   ```powershell
   .\sync_gx10.ps1
   ```
2. On the GX10 workstation, launch the unified perception and APF navigation engine:
   ```bash
   python3 launch_gx10.py
   ```
   *The robot will immediately begin autonomous exploration at a calm $0.015\text{ m/s}$, avoiding walls with a strict $50\text{ cm}$ safety buffer and pure in-place yaw rotations.*

---

### Step 4: Launch Voice AI Dispatch Console
1. In the `Voice_Cartesia_Mock/` directory:
   ```bash
   node server.mjs
   ```
2. Open `Voice_Cartesia_Mock/dashboard.html` in your browser to monitor real-time survivor interaction, audio waveform, and emergency triage data.

---

## 🛠️ Hardware Bill of Materials (BOM)

| Component | Quantity | Role |
| :--- | :---: | :--- |
| **ASUS Ascent GX10** | 1 | High-performance edge AI compute base station (Perception, APF, YOLO) |
| **Raspberry Pi 4 (4GB)** | 1 | Mobile robot controller (Motor PWM, camera streaming, ultrasonic IO) |
| **ESP32-C5 Development Boards** | 4 | Wi-Fi CSI human motion detection and RF localization nodes |
| **Logitech Brio USB Camera** | 1 | Forward-facing optical stream for Canny floor horizon & victim AI |
| **HC-SR04 Ultrasonic Sensors** | 3 | Left, Center, and Right obstacle proximity detection |
| **Kiwi 3-Wheel Omni Chassis** | 1 | Holonomic omni-directional rescue mobility |
| **L298N H-Bridge Driver** | 2 | Hardware PWM DC motor driver |

---

## 👥 Contributors & Team (CalHacks 13.0)
* **Huitae Han** — Autonomous Robotics, APF Navigation Engine, Edge Computer Vision & Multi-Sensor Fusion
* **Mohamed Cherif** — Wi-Fi CSI RF Sensing, ESP32-C5 Firmware, Signal Processing & 3D WebGL Digital Twin
* **Taehoon** — Voice AI Dispatcher, Cartesia Low-Latency Audio Streaming & Real-Time Incident Architecture
* **Hemakshi** — SpacetimeDB Real-Time State Synchronization, Incident Triage Console & System Integration
