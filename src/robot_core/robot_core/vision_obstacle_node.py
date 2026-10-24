import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import cv2
import numpy as np

class VisionObstacleNode(Node):
    """
    Edge AI / Computer Vision Node for ASUS Ascent GX10.
    Captures RGB camera frames, segments the floor ROI directly in front of the robot,
    and publishes estimated obstacle clearances (Front, Left, Right) to /obstacle_status.
    """
    def __init__(self):
        super().__init__('vision_obstacle_node')

        # Parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('edge_threshold_front', 0.12)  # Fraction of edge pixels considered blocked
        self.declare_parameter('edge_threshold_side', 0.10)

        cam_idx = self.get_parameter('camera_index').value
        self.cap = cv2.VideoCapture(cam_idx)

        if not self.cap.isOpened():
            self.get_logger().warn(
                f"Could not open camera on /dev/video{cam_idx}! Operating in MOCK CLEAR mode."
            )
            self.camera_available = False
        else:
            self.camera_available = True
            width = self.get_parameter('image_width').value
            height = self.get_parameter('image_height').value
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.get_logger().info(f"Camera opened successfully on /dev/video{cam_idx} ({width}x{height})")

        # Publisher for obstacle clearances:
        # x = front distance (m), y = left distance (m), z = right distance (m)
        self.obstacle_pub = self.create_publisher(Vector3, '/obstacle_status', 10)

        # 15 Hz processing loop
        self.timer = self.create_timer(1.0 / 15.0, self.process_frame)

    def process_frame(self):
        msg = Vector3()

        if not self.camera_available:
            # Mock mode: path is completely clear (2.0 meters clearance everywhere)
            msg.x = 2.0
            msg.y = 2.0
            msg.z = 2.0
            self.obstacle_pub.publish(msg)
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Failed to grab camera frame", throttle_duration_sec=3.0)
            msg.x = 2.0
            msg.y = 2.0
            msg.z = 2.0
            self.obstacle_pub.publish(msg)
            return

        h, w, _ = frame.shape

        # Region of Interest (ROI): Look at the lower 40% of the image (ground immediately ahead)
        roi_start_y = int(h * 0.60)
        roi = frame[roi_start_y:h, 0:w]

        # Convert to grayscale and blur to remove carpet/floor texture noise
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny edge detector: detects wall baseboards, obstacles, and maze corners
        edges = cv2.Canny(blurred, 50, 150)

        # Split the ROI into 3 vertical columns: Left (30%), Center (40%), Right (30%)
        col_left_end = int(w * 0.30)
        col_right_start = int(w * 0.70)

        roi_left = edges[:, 0:col_left_end]
        roi_center = edges[:, col_left_end:col_right_start]
        roi_right = edges[:, col_right_start:w]

        # Calculate edge density (percentage of white edge pixels) in each zone
        density_left = float(np.count_nonzero(roi_left)) / float(roi_left.size)
        density_center = float(np.count_nonzero(roi_center)) / float(roi_center.size)
        density_right = float(np.count_nonzero(roi_right)) / float(roi_right.size)

        thresh_front = self.get_parameter('edge_threshold_front').value
        thresh_side = self.get_parameter('edge_threshold_side').value

        # Convert edge density into approximate distance in meters (for Bug2 consumption):
        # Higher density = obstacle right in front of bumper (e.g. 0.20m)
        # Low density = open path (e.g. 1.50m)
        msg.x = 0.20 if density_center > thresh_front else 1.50
        msg.y = 0.20 if density_left > thresh_side else 1.50
        msg.z = 0.20 if density_right > thresh_side else 1.50

        self.obstacle_pub.publish(msg)

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
