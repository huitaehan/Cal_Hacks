"""
Edge AI & Computer Vision Node for Cal Hacks Autonomous Robotics.
Runs on ASUS Ascent GX10 Base Station.
Accepts USB Logitech webcam, Raspberry Pi MJPEG stream, or mock video.
Performs floor edge obstacle detection, YOLOv8/HOG victim identification,
pinhole 3D projection, and ultrasonic sensor fusion.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Point
from nav_msgs.msg import Odometry
import cv2
import numpy as np
import math
import time
from typing import Optional

try:
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:
    class CvBridgeError(Exception):
        pass

    class CvBridge:
        def imgmsg_to_cv2(self, img_msg, desired_encoding='bgr8'):
            dtype = np.uint8
            n_channels = 3
            if img_msg.encoding in ['mono8', '8UC1']:
                n_channels = 1
            elif img_msg.encoding in ['bgr8', 'rgb8', '8UC3']:
                n_channels = 3
            elif img_msg.encoding in ['bgra8', 'rgba8', '8UC4']:
                n_channels = 4
            im = np.frombuffer(img_msg.data, dtype=dtype).reshape((img_msg.height, img_msg.width, n_channels))
            if n_channels == 1:
                im = im.reshape((img_msg.height, img_msg.width))
            elif img_msg.encoding == 'rgb8' and desired_encoding == 'bgr8':
                im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
            return im

from robot_vision.detector import VictimDetector, Detection

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------
        self.declare_parameter('robot_id', '')               # e.g., 'robot1', 'robot2', or ''
        self.declare_parameter('camera_source', '/image_raw')# '/image_raw', '0', or 'http://...'
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('processing_fps', 20.0)
        self.declare_parameter('edge_threshold_front', 0.12)
        self.declare_parameter('edge_threshold_side', 0.10)
        self.declare_parameter('enable_victim_detection', True)
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('victim_conf_thresh', 0.35)
        self.declare_parameter('publish_target_location', True)
        self.declare_parameter('focal_length_px', 550.0)     # ~550px for 640x480 standard Logitech FOV (~60 deg)
        self.declare_parameter('person_real_height', 1.70)   # In meters
        self.declare_parameter('show_debug_window', False)

        self.robot_id = self.get_parameter('robot_id').value
        self.prefix = f"/{self.robot_id}" if self.robot_id else ""

        # Camera Initialization
        cam_src_param = str(self.get_parameter('camera_source').value)
        self.cap = None
        self.camera_available = False
        self._init_camera(cam_src_param)

        # Detector Initialization
        self.enable_victim = self.get_parameter('enable_victim_detection').value
        model_name = self.get_parameter('yolo_model').value
        self.detector = VictimDetector(model_name=model_name) if self.enable_victim else None

        # Robot Odometry State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.has_odom = False

        # Ultrasonic Sensor State (Fail-safe defaults to clear path)
        self.sonar_front = 2.0
        self.sonar_left = 2.0
        self.sonar_right = 2.0

        # Subscriptions
        odom_topic = f"{self.prefix}/odom" if self.prefix else "/odom"
        sonar_topic = f"{self.prefix}/ultrasonic_raw" if self.prefix else "/ultrasonic_raw"

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.sonar_sub = self.create_subscription(Vector3, sonar_topic, self._sonar_cb, 10)

        # Publishers
        obs_topic = f"{self.prefix}/obstacle_status" if self.prefix else "/obstacle_status"
        victim_topic = f"{self.prefix}/detected_person" if self.prefix else "/detected_person"
        target_topic = f"{self.prefix}/target_location" if self.prefix else "/target_location"

        self.obstacle_pub = self.create_publisher(Vector3, obs_topic, 10)
        self.victim_pub = self.create_publisher(Point, victim_topic, 10)
        self.target_pub = self.create_publisher(Point, target_topic, 10)

        # Processing Loop Timer
        fps = max(1.0, float(self.get_parameter('processing_fps').value))
        self.timer = self.create_timer(1.0 / fps, self.process_frame)

        self.get_logger().info(
            f"VisionNode ready for '{self.robot_id or 'global'}'. Source: '{cam_src_param}', Publishing to: {obs_topic}"
        )

    def _init_camera(self, cam_src: str):
        self.use_ros_image = False
        self.latest_frame = None
        self.bridge = CvBridge()

        # If camera source is a ROS 2 topic (e.g. '/image_raw' or 'ros')
        if cam_src.startswith('/') or cam_src == 'ros':
            topic_name = '/image_raw' if cam_src == 'ros' else cam_src
            self.use_ros_image = True
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE
            )
            self.image_sub = self.create_subscription(
                Image,
                topic_name,
                self._ros_image_cb,
                qos
            )
            self.camera_available = True
            self.get_logger().info(f"Subscribed to ROS 2 image topic: '{topic_name}' via cv_bridge")
            return

        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value

        try:
            # Check if camera source is an integer index (e.g. "0")
            if cam_src.isdigit():
                idx = int(cam_src)
                # Prefer V4L2 on Linux for Logitech cameras
                self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if not self.cap.isOpened():
                    self.cap = cv2.VideoCapture(idx)
                if self.cap.isOpened():
                    # Set FourCC to MJPG for 30+ FPS on USB 2.0
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            else:
                # Network URL (e.g. "http://192.168.1.101:5000/video")
                self.cap = cv2.VideoCapture(cam_src)

            if self.cap is not None and self.cap.isOpened():
                self.camera_available = True
                self.get_logger().info(f"Video capture opened successfully on: {cam_src}")
            else:
                self.camera_available = False
                self.get_logger().warn(
                    f"Could not open camera '{cam_src}'. Operating in FALLBACK/SENSOR-FUSION mode."
                )
        except Exception as e:
            self.camera_available = False
            self.get_logger().error(f"Error initializing camera '{cam_src}': {e}")

    def _ros_image_cb(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.camera_available = True
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error converting frame: {e}")

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def _sonar_cb(self, msg: Vector3):
        self.sonar_front = msg.x
        self.sonar_left = msg.y
        self.sonar_right = msg.z

    def process_frame(self):
        obs_msg = Vector3()

        if self.use_ros_image:
            frame = self.latest_frame
            if frame is None:
                # Awaiting first frame from ROS topic
                obs_msg.x = float(self.sonar_front)
                obs_msg.y = float(self.sonar_left)
                obs_msg.z = float(self.sonar_right)
                self.obstacle_pub.publish(obs_msg)
                return
        else:
            # If camera not connected, rely directly on ultrasonic sensors
            if not self.camera_available or self.cap is None:
                obs_msg.x = float(self.sonar_front)
                obs_msg.y = float(self.sonar_left)
                obs_msg.z = float(self.sonar_right)
                self.obstacle_pub.publish(obs_msg)
                return

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn("Failed to grab camera frame", throttle_duration_sec=4.0)
                obs_msg.x = float(self.sonar_front)
                obs_msg.y = float(self.sonar_left)
                obs_msg.z = float(self.sonar_right)
                self.obstacle_pub.publish(obs_msg)
                return

        h, w = frame.shape[:2]

        # ----------------------------------------------------
        # 1. Floor ROI Obstacle & Wall Detection
        # ----------------------------------------------------
        # Focus on lower 40% of the image (ground immediately ahead of bumper)
        roi_start_y = int(h * 0.60)
        roi = frame[roi_start_y:h, 0:w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Split into 3 columns: Left (30%), Center (40%), Right (30%)
        col_left_end = int(w * 0.30)
        col_right_start = int(w * 0.70)

        roi_left = edges[:, 0:col_left_end]
        roi_center = edges[:, col_left_end:col_right_start]
        roi_right = edges[:, col_right_start:w]

        den_l = float(np.count_nonzero(roi_left)) / float(max(1, roi_left.size))
        den_c = float(np.count_nonzero(roi_center)) / float(max(1, roi_center.size))
        den_r = float(np.count_nonzero(roi_right)) / float(max(1, roi_right.size))

        thresh_f = self.get_parameter('edge_threshold_front').value
        thresh_s = self.get_parameter('edge_threshold_side').value

        # Distance estimation: high edge density = wall/obstacle close to bumper
        vis_front = max(0.18, min(2.0, 0.28 / max(den_c, 0.05))) if den_c > thresh_f else 2.0
        vis_left  = max(0.18, min(2.0, 0.28 / max(den_l, 0.05))) if den_l > thresh_s else 2.0
        vis_right = max(0.18, min(2.0, 0.28 / max(den_r, 0.05))) if den_r > thresh_s else 2.0

        # Sensor Fusion: minimum of vision and ultrasonic for safety
        obs_msg.x = float(min(vis_front, self.sonar_front))
        obs_msg.y = float(min(vis_left, self.sonar_left))
        obs_msg.z = float(min(vis_right, self.sonar_right))
        self.obstacle_pub.publish(obs_msg)

        # ----------------------------------------------------
        # 2. Human Victim Detection (Full Frame)
        # ----------------------------------------------------
        distances = []
        bearings_deg = []
        detections = []

        if self.enable_victim and self.detector is not None:
            conf_th = self.get_parameter('victim_conf_thresh').value
            detections = self.detector.detect(frame, conf_thresh=conf_th)

            f_px = self.get_parameter('focal_length_px').value
            real_h = self.get_parameter('person_real_height').value

            for det in detections:
                # Distance via pinhole projection: d = (H_real * f) / h_px
                dist = (real_h * f_px) / max(det.h, 12.0)
                dist = max(0.3, min(dist, 6.0))
                distances.append(dist)

                # Bearing angle relative to camera optical axis
                px_offset = det.center_x - (w / 2.0)
                bearing_rad = math.atan2(px_offset, f_px)
                bearings_deg.append(math.degrees(bearing_rad))

                # Publish relative position in robot coordinate frame
                # Robot frame: +X forward, +Y left
                rel_x = dist * math.cos(bearing_rad)
                rel_y = -dist * math.sin(bearing_rad)

                victim_msg = Point()
                victim_msg.x = rel_x
                victim_msg.y = rel_y
                victim_msg.z = det.confidence
                self.victim_pub.publish(victim_msg)

                # World coordinate transformation and navigator update
                if self.get_parameter('publish_target_location').value and self.has_odom:
                    # Triangulate victim in global world coordinates
                    world_x = self.robot_x + rel_x * math.cos(self.robot_theta) - rel_y * math.sin(self.robot_theta)
                    world_y = self.robot_y + rel_x * math.sin(self.robot_theta) + rel_y * math.cos(self.robot_theta)

                    target_msg = Point()
                    target_msg.x = world_x
                    target_msg.y = world_y
                    target_msg.z = 0.0
                    self.target_pub.publish(target_msg)

                    self.get_logger().info(
                        f"[{self.robot_id or 'Robot'}] VICTIM SIGHTED at distance {dist:.2f}m! "
                        f"Target updated -> ({world_x:.2f}, {world_y:.2f})",
                        throttle_duration_sec=2.0
                    )

        # ----------------------------------------------------
        # 3. Optional Debug GUI Window
        # ----------------------------------------------------
        if self.get_parameter('show_debug_window').value:
            debug_frame = self.detector.draw_overlays(frame, detections, distances, bearings_deg) if self.detector else frame
            # Draw ROI box
            cv2.rectangle(debug_frame, (0, roi_start_y), (w, h), (255, 0, 0), 1)
            # HUD text
            hud = f"Front: {obs_msg.x:.2f}m | Left: {obs_msg.y:.2f}m | Right: {obs_msg.z:.2f}m"
            cv2.putText(debug_frame, hud, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(f"Vision Monitor [{self.robot_id or 'Base'}]", debug_frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.camera_available and self.cap is not None and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
