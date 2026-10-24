# Computer Vision & Sensor Perception Guide (`robot_core`)

This sub-guide covers the perception pipeline for search-and-rescue robotics: how to set up and stream from a **Logitech webcam**, which **vision models** to choose for detecting human victims and wall obstacles, and how to interface camera data with the APF navigation controller.

---

## 📌 Table of Contents
1. [Logitech Webcam Setup](#1-logitech-webcam-setup)
   - [Linux Hardware Verification](#linux-hardware-verification)
   - [Docker Device Pass-Through](#docker-device-pass-through)
   - [High-FPS OpenCV Driver Configuration](#high-fps-opencv-driver-configuration)
2. [What Model to Use for Victim Detection?](#2-what-model-to-use-for-victim-detection)
   - [Model Comparison: YOLOv8-Nano vs. MobileNet vs. HOG](#model-comparison-yolov8-nano-vs-mobilenet-vs-hog)
   - [Why YOLOv8-Nano is Recommended](#why-yolov8-nano-is-recommended)
   - [Quickstart: Person Detection with YOLOv8-Nano](#quickstart-person-detection-with-yolov8-nano)
3. [Obstacle & Wall Detection with Vision](#3-obstacle--wall-detection-with-vision)
   - [Floor ROI Canny Edge Segmentation](#floor-roi-canny-edge-segmentation)
   - [Fusing Vision with Ultrasonic Rangefinders](#fusing-vision-with-ultrasonic-rangefinders)
4. [Connecting Vision to the APF Navigator](#4-connecting-vision-to-the-apf-navigator)
   - [Triangulating 3D Victim Coordinates from 2D Bounding Boxes](#triangulating-3d-victim-coordinates-from-2d-bounding-boxes)
   - [Topic Flow: Eyes to Brain](#topic-flow-eyes-to-brain)
5. [Running the Vision Node](#5-running-the-vision-node)

---

## 1. Logitech Webcam Setup

Logitech webcams (C920, C270, C922, Brio, etc.) use standard **UVC (USB Video Class)** drivers, which are natively supported by the Linux kernel without requiring proprietary drivers.

### Linux Hardware Verification

Plug your Logitech webcam into a USB port on the ASUS Ascent GX10 host, then run:

```bash
# Check if Linux created the video devices
ls -l /dev/video*

# Install v4l-utils to inspect formats and resolutions
sudo apt install -y v4l-utils

# List connected cameras
v4l2-ctl --list-devices
```

You should see an entry like:
```text
HD Pro Webcam C920 (usb-0000:00:14.0-1):
    /dev/video0
    /dev/video1
```
> `/dev/video0` is the raw video stream; `/dev/video1` is the metadata stream. We use `/dev/video0`.

To see the supported resolutions and pixel formats:
```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

---

### Docker Device Pass-Through

By default, Docker containers **cannot access host USB devices**. To pass the Logitech webcam into your ROS 2 Docker container, specify the device flag when launching:

```bash
# Launch container with webcam access
sudo docker run -it \
  --net=host \
  --device=/dev/video0:/dev/video0 \
  -v /home/asus/calhacks:/workspace \
  -w /workspace \
  ros:humble bash
```

> **Tip (All USB Devices)**: If your camera index switches dynamically (e.g. `/dev/video2`), you can mount all host devices with `--privileged -v /dev:/dev`.

---

### High-FPS OpenCV Driver Configuration

In Linux/ROS 2, default OpenCV calls (`cv2.VideoCapture(0)`) can fall back to slow YUYV uncompressed streams (often capped at 5–10 FPS). 

For maximum frame rate (30–60 FPS) with low CPU usage, configure OpenCV to use the **V4L2 backend with MJPEG hardware decoding**:

```python
import cv2

# Open using V4L2 backend
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Request hardware MJPEG compression for high FPS
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

# Set resolution (320x240 or 640x480 is ideal for real-time embedded robotics)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30)

ret, frame = cap.read()
if ret:
    print(f"Captured frame: {frame.shape[1]}x{frame.shape[0]} at 30 FPS")
cap.release()
```

---

## 2. What Model to Use for Victim Detection?

For search-and-rescue at a hackathon, you need a model that:
1. Runs at **high FPS (> 20 FPS)** on ARM64 edge hardware (ASUS Ascent GX10).
2. Has **high accuracy on humans** in varied poses (standing, sitting, fallen).
3. Requires minimal setup without complex dependencies.

### Model Comparison: YOLOv8-Nano vs. MobileNet vs. HOG

| Model | Frame Rate (ARM64) | Model Size | Accuracy / False Positives | Implementation Complexity |
|:---|:---|:---|:---|:---|
| **YOLOv8-Nano (`yolov8n`)** ⭐ | **30 - 60+ FPS** | **6.2 MB** (3.2M params) | **Excellent** (State-of-the-art on partial / lying down poses) | **Very Low** (`pip install ultralytics`) |
| **MobileNet-SSD v2** | 20 - 35 FPS | 4.5 MB | Good on upright figures; struggles with unusual poses | Medium (requires loading Caffe/TFLite weights in OpenCV) |
| **OpenCV HOG + SVM** | 10 - 15 FPS | 0 MB (built-in) | Moderate; struggles with rotated poses or partial occlusions | Zero dependencies (built into `cv2`) |

---

### Why YOLOv8-Nano is Recommended

1. **Pre-trained on COCO**: Class ID `0` is `person`. It detects humans out of the box with zero additional training required.
2. **Anchor-Free Architecture**: Excels at detecting occluded people (e.g., someone partially hidden behind rubble or fallen down).
3. **Ultra-Simple Python API**: Runs inference in 3 lines of code and supports automatic confidence filtering.
4. **Exportable to TensorRT / ONNX**: If higher speeds are needed, it exports to ONNX/TensorRT with one flag (`model.export(format='onnx')`).

---

### Quickstart: Person Detection with YOLOv8-Nano

Inside your container or virtual environment:
```bash
pip install ultralytics
```

Python implementation:
```python
from ultralytics import YOLO
import cv2

# Load the nano model (auto-downloads 6MB weights on first run)
model = YOLO('yolov8n.pt')

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference for class 0 (person only), confidence threshold > 0.50
    results = model(frame, classes=[0], conf=0.50, verbose=False)

    for box in results[0].boxes:
        # Bounding box coordinates: [x1, y1, x2, y2]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        conf = float(box.conf[0])
        print(f"Victim sighted! Bounding Box: ({x1:.0f}, {y1:.0f}) to ({x2:.0f}, {y2:.0f}), Conf: {conf:.2f}")

    if cv2.waitKey(1) == 27:  # ESC to exit
        break

cap.release()
```

---

## 3. Obstacle & Wall Detection with Vision

While object detection models (like YOLO) detect discrete objects, flat cardboard walls often lack rich visual features. We use a complementary **classical computer vision approach** for walls:

### Floor ROI Canny Edge Segmentation
To detect where the floor ends and a cardboard/concrete wall begins:

```text
Full Camera Frame (320x240):
+---------------------------------------+
|                                       |  <- Ceiling / Distant background
|                                       |     (Ignored to avoid false edges)
+---------------------------------------+
|  ROI: Bottom 40% (Ground Ahead)       |
|  [Left: 30%]   [Center: 40%]  [Right] |  <- Canny edge density calculated here
+---------------------------------------+
```

1. **Crop Ground ROI**: Extract the bottom 40% of the image (`frame[int(h * 0.60):h, 0:w]`).
2. **Filter & Edge Detect**: Apply Gaussian Blur (5x5) and Canny Edge Detection (`cv2.Canny(blur, 50, 150)`). Baseboards and wall bottoms produce distinct horizontal and angled edges.
3. **Column Density Mapping**: Split the ROI into Left (30%), Center (40%), and Right (30%) columns.
4. **Metric Distance Estimation**: High edge density in the center column indicates a wall directly in front of the robot bumper ($\approx 0.20\text{m}$). Low edge density indicates an open path ($> 1.50\text{m}$).

---

### Fusing Vision with Ultrasonic Rangefinders

Ultrasonic rangefinders and optical cameras have complementary strengths:
- **Ultrasonics**: Work in total darkness, unaffected by low lighting or shadow patterns.
- **Vision**: Sees obstacles at angles where ultrasonic pings might deflect away.

**Fail-Safe Rule**:
```python
# Clearances published to /obstacle_status:
front_clearance = min(vision_front, ultrasonic_front)
left_clearance  = min(vision_left,  ultrasonic_left)
right_clearance = min(vision_right, ultrasonic_right)
```
If **either** sensor detects an obstacle, the robot halts forward drive and begins the APF streamline bypass.

---

## 4. Connecting Vision to the APF Navigator

### Triangulating 3D Victim Coordinates from 2D Bounding Boxes

When a victim is spotted in the camera frame, their pixel bounding box is converted into world $(X, Y)$ coordinates using the **pinhole camera model**:

```text
                     Victim
                       /\
                      /  \
                     /    \
     Distance (d)   /      \  Bounding Box Height (h_px)
                   /        \
                  /          \
              [Camera] ===> Image Sensor
```

#### Step 1: Distance Estimation
Using the ratio of real human height to pixel height:
$$\text{dist} = \frac{H_{\text{person, real}} \cdot f_{\text{focal}}}{h_{\text{px}}}$$
- $H_{\text{person, real}} \approx 1.70\text{m}$ (or $\approx 0.90\text{m}$ if seated/fallen)
- $f_{\text{focal}} \approx 280\text{px}$ (calibrated for $320 \times 240$ resolution)

#### Step 2: Bearing Angle
Using the horizontal offset of the box center from image center:
$$\phi = \arctan\left(\frac{X_{\text{box, center}} - W_{\text{image}} / 2}{f_{\text{focal}}}\right)$$

#### Step 3: World Coordinate Projection
Using current robot odometry $(X_{\text{robot}}, Y_{\text{robot}}, \theta)$:
$$X_{\text{victim}} = X_{\text{robot}} + \text{dist} \cdot \cos(\theta + \phi)$$
$$Y_{\text{victim}} = Y_{\text{robot}} + \text{dist} \cdot \sin(\theta + \phi)$$

Publishing $(X_{\text{victim}}, Y_{\text{victim}})$ to `/target_location` directly pulls the APF attraction vector toward the victim!

---

### Topic Flow: Eyes to Brain

```text
[Logitech Webcam]
       |
       v
[vision_obstacle_node]
       |
       +---> /obstacle_status (Vector3) ---> [navigator_node] ---> /cmd_vel (Twist)
       |     (x=front, y=left, z=right)           ^
       |                                          |
       +---> /target_location (Point) ------------+
             (Victim world coordinates)
```

---

## 5. Running the Vision Node

To launch the computer vision node with your Logitech webcam:

```bash
source install/setup.bash

# Run with default camera (/dev/video0)
ros2 run robot_core vision_obstacle_node

# Or specify a different camera index:
ros2 run robot_core vision_obstacle_node --ros-args -p camera_index:=1
```
