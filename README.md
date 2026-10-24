# Autonomous Search-and-Rescue Navigation (`robot_core`)

An autonomous navigation system for search-and-rescue robotics developed for Cal Hacks. The system combines real-time RF/sensor target triangulation with a custom **Streamline Artificial Potential Field (APF)** algorithm to navigate complex, unknown disaster environments (collapsed structures, multi-layered mazes, and office partitions) without requiring pre-mapped global grids.

---

## 📌 Table of Contents
1. [Project Overview & Architecture](#1-project-overview--architecture)
2. [The Navigation Algorithm](#2-the-navigation-algorithm)
   - [Why Artificial Potential Fields (APF)?](#why-artificial-potential-fields-apf)
   - [Mathematical Formulation](#mathematical-formulation)
   - [Overcoming Classic APF Failure Modes](#overcoming-classic-apf-failure-modes)
3. [System & Development Environment](#3-system--development-environment)
4. [Installation & Build](#4-installation--build)
5. [Running the System](#5-running-the-system)
   - [Testing Scenarios in Simulation](#testing-scenarios-in-simulation)
   - [Scenario Visual Maps & Edge Cases](#scenario-visual-maps--edge-cases)
   - [Terminal Map Visualizer](#terminal-map-visualizer)
6. [ROS 2 Topic Interfaces](#6-ros-2-topic-interfaces)
7. [Repository Structure](#7-repository-structure)

---

## 1. Project Overview & Architecture

In emergency search-and-rescue operations, robots must locate trapped victims through walls and rubble where GPS is unavailable and structural floorplans do not exist.

### System Pipeline
The robotic system is divided into three decoupled ROS 2 nodes:
1. **The Eyes (`sensor_node`)**: Triangulates the victim's location through walls using an ESP radar/RF array and publishes coordinates to `/target_location`. In simulation or testing, ultrasonic sensors/cameras report local wall proximity to `/obstacle_status`.
2. **The Brain (`navigator_node`)**: Subscribes to the target coordinates and sensor readings, computes real-time APF vector fields, and issues smooth motion commands to `/cmd_vel`.
3. **The Body (`motor_node` / `mock_robot_sim`)**: Differential drive base that executes `geometry_msgs/Twist` motor commands and publishes odometry to `/odom`.

```text
                  +----------------------------------+
                  |           THE EYES               |
                  |  ESP Sensor Array / Ultrasonics  |
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
- **Zero Pre-Mapping Required**: Operates strictly on instantaneous forces (O(1) constant time complexity at 10Hz).
- **Fluid & Organic Motion**: Pushes the robot smoothly around obstacles like a water droplet flowing down a canal.
- **Natural Target Arrival**: Decelerates smoothly upon approaching the target, avoiding overshooting or crashing into the victim.

---

### Mathematical Formulation

```text
               Goal Attraction: F_att (pulls toward target)
                           \
                            \     +---------+
                             v   /| Wall    |
             [Robot] ---------->  | Barrier |
                     ^           \|         |
                    /             +---------+
                   /
     Obstacle Repulsion: F_rep (pushes away from danger)
```

#### 1. Target Attraction Force (The Pull)
Pulls the robot toward the victim's (X, Y) coordinates with Euclidean distance scaling and terminal arrival deceleration:

```text
Distance to Goal:
  dist = sqrt((X_target - X_robot)^2 + (Y_target - Y_robot)^2)

Attraction Magnitude (capped to avoid overpowering obstacle sensors):
  F_mag = min(K_att * dist, F_max)

World Attraction Vector:
  F_att_world_x = F_mag * (X_target - X_robot) / dist
  F_att_world_y = F_mag * (Y_target - Y_robot) / dist
```

Transformed into the robot's local frame (+X = forward, +Y = left) using robot yaw (θ):

```text
F_att_robot_x =  F_att_world_x * cos(θ) + F_att_world_y * sin(θ)
F_att_robot_y = -F_att_world_x * sin(θ) + F_att_world_y * cos(θ)
```

#### 2. Steep Canal Embankment Repulsion (The Push)
Obstacle pushback rises sharply when an object breaches the danger threshold (danger_zone = 0.75m):

```text
For distance d < danger_zone:
  ratio = (danger_zone - d) / max(d, 0.04)
  F_rep = min( K_rep * (ratio ^ 1.8), F_rep_max )
```

Sensors apply orthogonal repulsion vectors in the robot's frame:
- **Front sensor**: pushes backward (`-F_rep_front` along X)
- **Left sensor**: pushes rightward (`-F_rep_left` along Y)
- **Right sensor**: pushes leftward (`+F_rep_right` along Y)

#### 3. Smooth Streamline Bypass (Vortex Potential)
Standard APF suffers from deadlock when facing a flat wall head-on (forward attraction and backward repulsion cancel to 0).
To solve this without discrete stop-and-pivot kicks, we introduce a **streamline tangent force**:

```text
When front wall detected (F_rep_front > 0.05):
  F_bypass_mag = max(F_rep_front * 1.8, 1.4)

  # Desired streamline flow along wall in world coordinates:
  # (0, target_world_y * F_bypass_mag)

  # Transformed into robot frame:
  F_bypass_robot_x = (target_world_y * F_bypass_mag) * sin(θ)  # Forward drive once aligned!
  F_bypass_robot_y = (target_world_y * F_bypass_mag) * cos(θ)  # Steering torque, drops to 0 when aligned!
```

---

### Overcoming Classic APF Failure Modes

| Classic APF Problem | Cause | Our Solution |
|:---|:---|:---|
| **Head-on Deadlock** | Net F_x = 0 against flat wall | **Canal Streamline Flow**: Natural tangent potential rotates robot smoothly along wall surface without stopping. |
| **Pacing / Oscillation** | Bouncing between opposing forces | **Pacing Trap Detector**: Tracks net displacement over a 2.5s window (< 0.35m). Reverses bypass direction for a locked 4.0s window. |
| **U-Trap Dead Ends** | Robot stuck in concave pocket | **Boredom Timer**: If robot stalls for > 3.0s while far from target, triggers a forced 90° escape pivot. |
| **Corridor Misdirection** | Exploring dead-ends repeatedly | **Corridor Exploration Memory**: Discretized grid tracking visit frequency strictly within local channel width (dx < 0.35m). Steers away from traversed paths. |
| **Boundary Collisions** | Driving into ceiling/floor | **Hard Boundary Clamping**: Boundary-aware bypass overrides heading when near arena limits (Y >= 2.45m -> DOWN, Y <= 0.55m -> UP). |
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
| **`triple`** | `-p scenario:=triple` | **Gauntlet**: 3 alternating wall layers (Up -> Down -> Up). Verifies sequential maze navigation. |
| **`office`** | `-p scenario:=office` | **Building Floorplan**: Hallway entrance (Y = 1.3m) and interior room doorways (X = 2.6m). Realistic search-and-rescue. |
| **`clutter`** | `-p scenario:=clutter` | **Debris & Pillars**: Multiple rubble columns. Tests fluid avoidance around standalone obstacles without corners snagging. |
| **`u_trap`** | `-p scenario:=u_trap` | **Concave Dead End**: U-shaped pocket directly in front of target. Tests the Boredom Timer escape maneuver. |
| **`corridor`** | `-p scenario:=corridor` | **Doorway Gap**: Narrow center doorway. Tests passing through close side walls without false braking. |
| **`wall`** | `-p scenario:=wall` | **Single Wall**: Basic obstacle avoidance and target acquisition. |

---

### Scenario Visual Maps & Edge Cases

Below are the ASCII floorplans for each scenario showing starting positions `[R]`, victim target `[T]`, physical walls `#`, openings, and autonomous navigation paths:

#### 1. `multilayer` — S-Curve Alternating Slalom (Slit Top, Slit Bottom)
Tests navigating through alternating vertical partitions without getting stuck on dead-end ceiling/floor junctions.
```text
+----------------------------------+
|           #         #            |
|           #         #            |  <-- Slit at Top (Y > 2.1m)
|           #                      |
|           #    /\                |
|   [R] --> #   /  \               |
|           #  /    \              |
|           # /      \     [T]     |
|                     #            |
|                     #            |  <-- Slit at Bottom (Y < 0.9m)
|                     #            |
+----------------------------------+
```

#### 2. `triple` — The 3-Layer Gauntlet (Up -> Down -> Up)
Tests repetitive corridor transitions, verifying that exploration memory resets between consecutive walls.
```text
+----------------------------------+
|        #                 #       |
|        #        #        #       |  <-- Slits at Top (Y > 2.1m)
|        #        #        #       |
|        #   /\   #        #   [T] |
|   [R]->#  /  \  #       /#  /    |
|        # /    \ #      / # /     |
|        #       \#     /  #/      |
|                 #    /           |
|                 #   /            |  <-- Slit at Bottom (Y < 0.9m)
|                 #                |
+----------------------------------+
```

#### 3. `office` — Realistic Search-and-Rescue Multi-Room Floorplan
Models an earthquake-damaged office. The robot must travel down the hallway, enter through the doorway, turn into Room 1, pass through an interior door into Room 2, and dock at the victim.
```text
+----------------------------------+
| Room 1           | Room 2        |
|                  |     [T]       |
|             /\   |      ^        |
|            /  \  . <----+--------|-- Interior Doorway (Y = 1.9m - 2.5m)
|           /    \ |               |
|==========+      ++===============|-- Hallway Wall (Y = 1.3m)
|          |      |                |
|   [R]----+----->                 |-- Hallway Entrance (X = 1.8m - 2.6m)
+----------------------------------+
```

#### 4. `clutter` — Debris & Pillar Field
Tests fluid avoidance through randomly scattered rubble columns without getting hooked on corners or oscillating between closely spaced obstacles.
```text
+----------------------------------+
|        ##              ##        |
|        ##              ##        |
|           \                      |
|            \    ##          [T]  |
|   [R] ------>   ##          /    |
|                   \        /     |
|                    \      /      |
|        ##           \  ##        |
|        ##            \ ##        |
+----------------------------------+
```

#### 5. `u_trap` — Concave Trap (Local Minima Escape)
The target is behind a deep U-shaped pocket. Standard APF freezes inside the U-pocket. The Boredom Timer detects the stall (> 3.0s) and forces a 90° escape turn to exit the trap.
```text
+----------------------------------+
|             +--------+           |
|             |        |           |
|             |  EXIT  |           |
|             |   ^    |           |
|   [R] =====>|   |    |    [T]    |  <-- Trapped inside local minimum!
|             |   +--+ |           |      Boredom Timer triggers escape
|             |      | |           |
|             +--------+           |
+----------------------------------+
```

#### 6. `corridor` — Doorway Gap
A narrow door opening in the center of a partition wall. Tests passing through close side obstacles without false side-braking.
```text
+----------------------------------+
|                 #                |
|                 #                |
|                 #                |
|   [R] --------> | --------> [T]  |  <-- Doorway Gap (1.1m - 1.8m)
|                 #                |
|                 #                |
|                 #                |
+----------------------------------+
```

#### 7. `wall` — Single Obstacle Bypass
Basic test verifying streamline tangent bypass around a vertical partition wall.
```text
+----------------------------------+
|                                  |
|                 #                |
|           /\    #                |
|   [R] -->/  \----------> [T]     |  <-- Streamline canal flow around edge
|                 #                |
|                 #                |
+----------------------------------+
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
| `/target_location` | `geometry_msgs/Point` | Input to Navigator | Victim's world coordinates (X, Y) published by sensor array. |
| `/odom` | `nav_msgs/Odometry` | Input to Navigator | Current robot pose (X, Y, θ) from wheel encoders. |
| `/obstacle_status` | `geometry_msgs/Vector3` | Input to Navigator | Ultrasonic/vision obstacle distance (`x` = front, `y` = left, `z` = right in meters). |
| `/cmd_vel` | `geometry_msgs/Twist` | Output from Navigator | Velocity drive commands (`linear.x` in m/s, `angular.z` in rad/s). |

---

## 7. Repository Structure

```text
calhacks/
├── README.md                                  # Complete system documentation & visual maps
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
