# Autonomous Search-and-Rescue Navigation (`robot_core`)

An autonomous navigation system for search-and-rescue robotics developed for Cal Hacks. The system combines real-time RF/sensor target triangulation with a custom **Streamline Artificial Potential Field (APF)** algorithm to navigate complex, unknown disaster environments (collapsed structures, multi-layered mazes, and office partitions) without requiring pre-mapped global grids.

---

## 📌 Table of Contents
1. [Project Overview & Architecture](#project-overview--architecture)
2. [The Navigation Algorithm](#the-navigation-algorithm)
   - [Why Artificial Potential Fields (APF)?](#why-artificial-potential-fields-apf)
   - [Mathematical Formulation](#mathematical-formulation)
   - [Overcoming Classic APF Failure Modes](#overcoming-classic-apf-failure-modes)
3. [System & Development Environment](#system--development-environment)
4. [Installation & Build](#installation--build)
5. [Running the System](#running-the-system)
   - [Testing Scenarios in Simulation](#testing-scenarios-in-simulation)
   - [Terminal Map Visualizer](#terminal-map-visualizer)
6. [ROS 2 Topic Interfaces](#ros-2-topic-interfaces)
7. [Repository Structure](#repository-structure)

---

## 1. Project Overview & Architecture

In emergency search-and-rescue operations, robots must locate trapped victims through walls and rubble where GPS is unavailable and structural maps do not exist.

### System Pipeline
The robotic system is divided into three decoupled ROS 2 nodes:
1. **The Eyes (`sensor_node`)**: Triangulates the victim's location through walls using an ESP radar/RF array and publishes coordinates to `/target_location`. In simulation or testing, ultrasonic sensors/cameras report local wall proximity to `/obstacle_status`.
2. **The Brain (`navigator_node`)**: Subscribes to the target coordinates and sensor readings, computes real-time APF vector fields, and issues smooth motion commands to `/cmd_vel`.
3. **The Body (`motor_node` / `mock_robot_sim`)**: Differential drive base that executes `geometry_msgs/Twist` motor commands and publishes odometry to `/odom`.

```
                  +----------------------------------+
                  |           THE EYES               |
                  |  ESP Sensor Array / Ultrasonics   |
                  +-----------------+----------------+
                                    |
            /target_location (Point)|  /obstacle_status (Vector3)
                                    v
                  +----------------------------------+
                  |           THE BRAIN              |
                  |        navigator_node.py         |
                  |   (Streamline APF + Memory)      |
                  +-----------------+----------------+
                                    |
                    /cmd_vel (Twist)|
                                    v
                  +----------------------------------+
                  |           THE BODY               |
                  |    Motor Node / Mock Sim         |
                  +-----------------+----------------+
                                    | /odom (Odometry)
                                    +---------------> (Feedback Loop)
```

---

## 2. The Navigation Algorithm

### Why Artificial Potential Fields (APF)?
Traditional navigation stacks (like Nav2 / Global A* / Dijkstra) require **pre-built global occupancy grids** (SLAM). In dynamic disaster sites:
- Mapping an entire floor before moving is too slow for time-critical rescue missions.
- Global path planners consume heavy CPU/RAM, making them difficult to run on lightweight edge processors.

**Why APF is ideal for this mission:**
- **Zero Pre-Mapping Required**: Operates strictly on instantaneous forces ($O(1)$ constant time complexity at 10Hz).
- **Fluid & Organic Motion**: Pushes the robot smoothly around obstacles like a water droplet flowing down a canal.
- **Natural Target Arrival**: Decelerates smoothly upon approaching the target, avoiding overshooting or crashing into the victim.

---

### Mathematical Formulation

#### 1. Target Attraction Force (The Pull)
Pulls the robot toward the victim's $(X, Y)$ location:
$$\vec{F}_{\text{att}} = \min\left(K_{\text{att}} \cdot \|\vec{p}_{\text{target}} - \vec{p}_{\text{robot}}\|, F_{\text{att, max}}\right) \cdot \frac{\vec{p}_{\text{target}} - \vec{p}_{\text{robot}}}{\|\vec{p}_{\text{target}} - \vec{p}_{\text{robot}}\|}$$

The force is rotated into the robot's local frame $(+X = \text{forward}, +Y = \text{left})$ using the robot's heading $\theta$:
$$F_{\text{att, robot, x}} = F_{\text{att, world, x}} \cos\theta + F_{\text{att, world, y}} \sin\theta$$
$$F_{\text{att, robot, y}} = -F_{\text{att, world, x}} \sin\theta + F_{\text{att, world, y}} \cos\theta$$

#### 2. Steep Canal Embankment Repulsion (The Push)
Obstacle pushback rises sharply when an object breaches the danger threshold ($d_{\text{danger}} = 0.75\text{m}$):
$$F_{\text{rep}}(d) = \min\left(K_{\text{rep}} \cdot \left(\frac{d_{\text{danger}} - d}{d}\right)^{1.8}, F_{\text{rep, max}}\right) \quad \text{for } d < d_{\text{danger}}$$

Sensors apply orthogonal repulsion in the robot's frame:
- Front sensor $\implies -F_{\text{rep, front}} \cdot \hat{x}$
- Left sensor $\implies -F_{\text{rep, left}} \cdot \hat{y}$
- Right sensor $\implies +F_{\text{rep, right}} \cdot \hat{y}$

#### 3. Smooth Streamline Bypass (Vortex Potential)
Standard APF suffers from deadlock when facing a flat wall head-on ($F_{\text{att, x}} \approx F_{\text{rep, x}}$ cancels to zero). 
To solve this without discrete stop-and-pivot kicks, we introduce a **streamline tangent force**:
- When approaching a wall head-on ($F_{\text{rep, front}} > 0.05$), the robot projects a lateral flow force:
  $$\vec{F}_{\text{bypass, world}} = (0, \text{target\_world\_y} \cdot F_{\text{mag}})$$
- Transformed to the robot frame:
  $$F_{\text{bypass, robot, x}} = (\text{target\_world\_y} \cdot F_{\text{mag}}) \sin\theta \quad (\text{forward momentum once turned})$$
  $$F_{\text{bypass, robot, y}} = (\text{target\_world\_y} \cdot F_{\text{mag}}) \cos\theta \quad (\text{turning torque, drops to zero once aligned})$$

---

### Overcoming Classic APF Failure Modes

| Classic APF Problem | Cause | Our Solution |
|:---|:---|:---|
| **Head-on Deadlock** | Net $F_x = 0$ against flat wall | **Canal Streamline Flow**: Natural tangent potential rotates robot smoothly along wall surface without stopping. |
| **Pacing / Oscillation** | Bouncing between opposing forces | **Pacing Trap Detector**: Tracks net displacement over a 2.5s window ($< 0.35\text{m}$). Reverses bypass direction for a locked 4.0s window. |
| **U-Trap Dead Ends** | Robot stuck in concave pocket | **Boredom Timer**: If robot stalls for $> 3.0\text{s}$ while far from target, triggers a forced $90^\circ$ escape pivot. |
| **Corridor Misdirection** | Exploring dead-ends repeatedly | **Corridor Exploration Memory**: Discretized grid tracking visit frequency strictly within local channel width ($dx < 0.35\text{m}$). Steers away from traversed paths. |
| **Boundary Collisions** | Driving into ceiling/floor | **Hard Boundary Clamping**: Boundary-aware bypass overrides heading when near arena limits ($Y \ge 2.45\text{m} \implies \text{DOWN}$, $Y \le 0.55\text{m} \implies \text{UP}$). |
| **Wall Penetration** | Kinematic discrete stepping | **Continuous Collision Detection (CCD)**: Cross-product segment intersection and multi-obstacle clearance checks prevent passing through walls. |

---

## 3. System & Development Environment

- **Host Architecture**: ASUS Ascent GX10 (ARM64)
- **Host OS**: Ubuntu / Linux
- **Remote Access**: Tailscale Remote-SSH
- **Containerization**: Docker (`ros:humble` base image)
- **ROS 2 Distribution**: ROS 2 Humble Hawksbill
- **Build System**: `colcon` with `--symlink-install`
- **Container Workspace Mapping**: `/home/asus/calhacks` mapped to `/workspace` inside Docker.

---

## 4. Installation & Build

### 1. Enter the Docker Container
From the host system:
```bash
# Start container (if not already running)
sudo docker run -it --net=host -v /home/asus/calhacks:/workspace -w /workspace ros:humble bash

# Or attach to an already running container
sudo docker exec -it $(sudo docker ps -q | head -n 1) bash
```

### 2. Build the ROS 2 Workspace
Inside the container:
```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

> **Note**: `--symlink-install` links the Python source files directly to the install directory. When you edit `.py` files in `src/`, changes take effect immediately upon restarting the node without requiring a full rebuild.

---

## 5. Running the System

Open **two terminal sessions** inside the container:

### Terminal 1: Launch the Simulator
```bash
source /workspace/install/setup.bash
ros2 run robot_core mock_robot_sim --ros-args -p scenario:=multilayer
```

### Terminal 2: Launch the Navigator
```bash
source /workspace/install/setup.bash
ros2 run robot_core navigator_node
```

---

### Testing Scenarios in Simulation

Select any scenario using the `scenario` parameter:

| Scenario Name | Command Flag | Layout & Test Objective |
|:---|:---|:---|
| **`multilayer`** | `-p scenario:=multilayer` | **S-Curve Slalom**: Wall 1 opening at top, Wall 2 opening at bottom. Tests seamless corridor transitions. |
| **`triple`** | `-p scenario:=triple` | **Gauntlet**: 3 alternating wall layers (Up $\to$ Down $\to$ Up). Verifies sequential maze navigation. |
| **`office`** | `-p scenario:=office` | **Building Floorplan**: Hallway entrance ($Y=1.3\text{m}$) and interior room doorways ($X=2.6\text{m}$). Realistic search-and-rescue. |
| **`clutter`** | `-p scenario:=clutter` | **Debris & Pillars**: Multiple rubble columns. Tests fluid avoidance around standalone obstacles without corners snagging. |
| **`u_trap`** | `-p scenario:=u_trap` | **Concave Dead End**: U-shaped pocket directly in front of target. Tests the Boredom Timer escape maneuver. |
| **`corridor`** | `-p scenario:=corridor` | **Doorway Gap**: Narrow center doorway. Tests passing through close side walls without false braking. |
| **`wall`** | `-p scenario:=wall` | **Single Wall**: Basic obstacle avoidance and target acquisition. |

**Example**: Run the 3-layer gauntlet:
```bash
ros2 run robot_core mock_robot_sim --ros-args -p scenario:=triple
```

---

### Terminal Map Visualizer

The simulator includes a high-speed ASCII real-time map renderer in the terminal:

```text
========================================
 🤖 MOCK ROBOT APF SIMULATOR (MULTILAYER)
========================================
+----------------------------------+
|           #         #            |
|           #         #            |
|           #         #            |
|           #         #            |
|           #         #            |
|           #         #        T   |
|   >       #                      |
|           #                      |
|           #                      |
|                                  |
+----------------------------------+
 Robot Pose:  X:  0.85m, Y:  1.12m | Heading:  18°
 Goal Dist:    3.27m  | Target: (4.0, 2.0)
 Clearances:  Front: 0.85m | Left: 1.88m | Right: 0.96m
 Motors Cmd:  linear.x =  0.25 m/s | angular.z =  0.12 rad/s
========================================
```

- `>` `^` `<` `v`: Robot heading indicator
- `#`: Physical wall obstacles
- `T`: Human victim target location
- `+----+`: Sealed physical arena perimeter walls

---

## 6. ROS 2 Topic Interfaces

| Topic | Message Type | Direction | Description |
|:---|:---|:---|:---|
| `/target_location` | `geometry_msgs/Point` | Input to Navigator | Victim's world coordinates $(X, Y)$ published by sensor array. |
| `/odom` | `nav_msgs/Odometry` | Input to Navigator | Current robot pose $(X, Y, \theta)$ from wheel encoders. |
| `/obstacle_status` | `geometry_msgs/Vector3` | Input to Navigator | Ultrasonic/vision obstacle distance (`x`=front, `y`=left, `z`=right in meters). |
| `/cmd_vel` | `geometry_msgs/Twist` | Output from Navigator | Velocity drive commands (`linear.x` in m/s, `angular.z` in rad/s). |

---

## 7. Repository Structure

```text
calhacks/
├── README.md                                  # Complete system documentation
├── src/
│   └── robot_core/
│       ├── package.xml                        # ROS 2 package configuration
│       ├── setup.py                           # Python entry points & package metadata
│       ├── launch/                            # Launch files
│       └── robot_core/
│           ├── __init__.py
│           ├── navigator_node.py              # APF Brain (attraction, repulsion, memory, bypass)
│           ├── mock_robot_sim.py              # Physics engine, CCD, raycaster & ASCII renderer
│           └── vision_obstacle_node.py        # Vision/camera sensor processing node
├── install/                                   # Colcon symlink-installed artifacts
└── build/                                     # Colcon intermediate build files
```
