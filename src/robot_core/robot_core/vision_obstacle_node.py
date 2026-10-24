import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point
from nav_msgs.msg import Odometry
import cv2
import numpy as np
import math

class VisionObstacleNode(Node):
    """
    Edge AI & Computer Vision Node for ASUS Ascent GX10.
    1. Obstacle Detection: Segments the floor Region of Interest (ROI), detects wall
       edges and obstacles, and publishes clearances to /obstacle_status.
    2. Person Detection: Uses OpenCV People Detection to recognize human victims,
       estimates their distance and bearing, and publishes to /detected_person
       and /target_location.
    """
    def __init__(self):
        super().__init__('vision_obstacle_node')

        # Parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('edge_threshold_front', 0.12)
        self.declare_parameter('edge_threshold_side', 0.10)
        self.declare_parameter('enable_person_detection', True)
        self.declare_parameter('publish_target_location', True)
        self.declare_parameter('focal_length_px', 280.0)  # Approximate focal length for 320x240 webcam
        self.declare_parameter('person_real_height', 1.70) # Average adult height in meters

        cam_idx = self.get_parameter('camera_index').value
        self.cap = cv2.VideoCapture(cam_idx)

        if not self.cap.isOpened():
            self.get_logger().warn(
                f"Camera on /dev/video{cam_idx} not detected. Operating in SIMULATED/PASS-THROUGH mode."
            )
            self.camera_available = False
        else:
            self.camera_available = True
            width = self.get_parameter('image_width').value
            height = self.get_parameter('image_height').value
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.get_logger().info(f"Camera opened successfully on /dev/video{cam_idx} ({width}x{height})")

        # Initialize OpenCV HOG Person Detector
        self.enable_person_det = self.get_parameter('enable_person_detection').value
        if self.enable_person_det:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.get_logger().info("HOG Person Detector initialized for victim identification.")

        # Robot Pose Tracking (from /odom) for world-coordinate triangulation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.has_odom = False
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Optional raw ultrasonic subscription for sensor fusion
        self.ultrasonic_front = 2.0
        self.ultrasonic_left = 2.0
        self.ultrasonic_right = 2.0
        self.ultrasonic_sub = self.create_subscription(Vector3, '/ultrasonic_raw', self.ultrasonic_callback, 10)

        # Publishers
        # /obstacle_status: Vector3 (x=front_dist, y=left_dist, z=right_dist in meters)
        self.obstacle_pub = self.create_publisher(Vector3, '/obstacle_status', 10)
        # /detected_person: Point (x=rel_x, y=rel_y, z=confidence)
        self.person_pub = self.create_publisher(Point, '/detected_person', 10)
        # /target_location: Point (world_x, world_y, 0.0) consumed by navigator_node
        self.target_pub = self.create_publisher(Point, '/target_location', 10)

        # 15 Hz processing loop
        self.timer = self.create_timer(1.0 / 15.0, self.process_frame)

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def ultrasonic_callback(self, msg: Vector3):
        self.ultrasonic_front = msg.x
        self.ultrasonic_left = msg.y
        self.ultrasonic_right = msg.z

    def process_frame(self):
        obs_msg = Vector3()

        if not self.camera_available:
            # Fallback to ultrasonic readings or clear path (2.0m)
            obs_msg.x = self.ultrasonic_front
            obs_msg.y = self.ultrasonic_left
            obs_msg.z = self.ultrasonic_right
            self.obstacle_pub.publish(obs_msg)
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Failed to grab camera frame", throttle_duration_sec=3.0)
            obs_msg.x = self.ultrasonic_front
            obs_msg.y = self.ultrasonic_left
            obs_msg.z = self.ultrasonic_right
            self.obstacle_pub.publish(obs_msg)
            return

        h, w, _ = frame.shape

        # ----------------------------------------------------
        # 1. Floor ROI Obstacle & Wall Detection
        # ----------------------------------------------------
        # Region of Interest: lower 40% of the image (ground immediately ahead)
        roi_start_y = int(h * 0.60)
        roi = frame[roi_start_y:h, 0:w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # 3 vertical zones: Left (30%), Center (40%), Right (30%)
        col_left_end = int(w * 0.30)
        col_right_start = int(w * 0.70)

        roi_left = edges[:, 0:col_left_end]
        roi_center = edges[:, col_left_end:col_right_start]
        roi_right = edges[:, col_right_start:w]

        density_left = float(np.count_nonzero(roi_left)) / float(roi_left.size)
        density_center = float(np.count_nonzero(roi_center)) / float(roi_center.size)
        density_right = float(np.count_nonzero(roi_right)) / float(roi_right.size)

        thresh_front = self.get_parameter('edge_threshold_front').value
        thresh_side = self.get_parameter('edge_threshold_side').value

        # Calculate estimated visual distance: smooth exponential decay with edge density
        vis_front = max(0.15, min(2.0, 0.25 / max(density_center, 0.05))) if density_center > thresh_front else 2.0
        vis_left  = max(0.15, min(2.0, 0.25 / max(density_left, 0.05))) if density_left > thresh_side else 2.0
        vis_right = max(0.15, min(2.0, 0.25 / max(density_right, 0.05))) if density_right > thresh_side else 2.0

        # Sensor fusion: take minimum distance between Vision and Ultrasonic for fail-safe safety
        obs_msg.x = min(vis_front, self.ultrasonic_front)
        obs_msg.y = min(vis_left, self.ultrasonic_left)
        obs_msg.z = min(vis_right, self.ultrasonic_right)
        self.obstacle_pub.publish(obs_msg)

        # ----------------------------------------------------
        # 2. Person / Human Victim Detection (Full Frame)
        # ----------------------------------------------------
        if self.enable_person_det:
            # Resize for faster CPU HOG inference on embedded compute
            small_frame = cv2.resize(frame, (160, 120))
            boxes, weights = self.hog.detectMultiScale(small_frame, winStride=(8, 8), padding=(4, 4), scale=1.05)

            if len(boxes) > 0:
                # Find the largest detected person (highest confidence)
                best_idx = int(np.argmax([w[0] if isinstance(w, (list, np.ndarray)) else w for w in weights]))
                bx, by, bw, bh = boxes[best_idx]
                conf = float(weights[best_idx]) if not isinstance(weights[best_idx], (list, np.ndarray)) else float(weights[best_idx][0])

                # Scale box back to original resolution
                scale_x = w / 160.0
                scale_y = h / 120.0
                orig_bw = bw * scale_x
                orig_bh = bh * scale_y
                center_x = (bx + bw / 2.0) * scale_x

                # Distance estimation via pinhole camera model:
                # distance = (real_height * focal_length) / bbox_height
                f_px = self.get_parameter('focal_length_px').value
                real_h = self.get_parameter('person_real_height').value
                est_dist = (real_h * f_px) / max(orig_bh, 10.0)
                est_dist = max(0.3, min(est_dist, 6.0))

                # Bearing angle relative to camera center (FOV approx 60 degrees)
                pixel_offset = center_x - (w / 2.0)
                bearing_rad = math.atan2(pixel_offset, f_px)

                # Relative person position in robot frame:
                rel_x = est_dist * math.cos(bearing_rad)
                rel_y = -est_dist * math.sin(bearing_rad)  # Camera +X is right, robot +Y is left

                person_msg = Point()
                person_msg.x = rel_x
                person_msg.y = rel_y
                person_msg.z = conf
                self.person_pub.publish(person_msg)

                # If enabled, transform to world coordinates and publish to /target_location
                if self.get_parameter('publish_target_location').value and self.has_odom:
                    world_x = self.robot_x + rel_x * math.cos(self.robot_theta) - rel_y * math.sin(self.robot_theta)
                    world_y = self.robot_y + rel_x * math.sin(self.robot_theta) + rel_y * math.cos(self.robot_theta)

                    target_msg = Point()
                    target_msg.x = world_x
                    target_msg.y = world_y
                    target_msg.z = 0.0
                    self.target_pub.publish(target_msg)
                    self.get_logger().info(
                        f"Victim sighted at distance {est_dist:.2f}m! Target updated to ({world_x:.2f}, {world_y:.2f})",
                        throttle_duration_sec=2.0
                    )

    def destroy_node(self):
        if self.camera_available and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
