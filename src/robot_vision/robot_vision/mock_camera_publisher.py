#!/usr/bin/env python3
"""
Mock Sensor Publisher for Offline Testing.
Publishes synthetic /image_raw and /ultrasonic_distance streams at 15 Hz
so you can test visual_wall_detector and navigator_node without needing
any physical camera, sensors, or Raspberry Pi connected.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32
import cv2
import numpy as np
import math
import time

try:
    from cv_bridge import CvBridge
except ImportError:
    class CvBridge:
        def cv2_to_imgmsg(self, cv_image, encoding='bgr8'):
            msg = Image()
            msg.height = cv_image.shape[0]
            msg.width = cv_image.shape[1]
            msg.encoding = encoding
            msg.step = cv_image.shape[1] * 3
            msg.data = cv_image.tobytes()
            return msg


class MockSensorPublisher(Node):
    def __init__(self):
        super().__init__('mock_sensor_publisher')

        self.pub_image = self.create_publisher(Image, '/image_raw', 10)
        self.pub_range = self.create_publisher(Range, '/ultrasonic_distance', 10)
        self.bridge = CvBridge()

        self.width = 640
        self.height = 480
        self.step_count = 0

        # Publish loop at 15 Hz
        self.timer = self.create_timer(1.0 / 15.0, self.publish_mock_data)
        self.get_logger().info("MockSensorPublisher active: Streaming synthetic /image_raw and /ultrasonic_distance at 15 Hz.")

    def publish_mock_data(self):
        self.step_count += 1
        t = self.step_count * 0.08

        # ----------------------------------------------------
        # 1. Generate Synthetic Camera Frame
        # ----------------------------------------------------
        frame = np.full((self.height, self.width, 3), 210, dtype=np.uint8)  # Light room background

        # Lower half: Floor plane (darker grey)
        floor_y = int(self.height * 0.50)
        frame[floor_y:self.height, :] = (120, 120, 120)

        # Draw baseboard horizon line
        cv2.line(frame, (0, floor_y), (self.width, floor_y), (40, 40, 40), 2)

        # Animated obstacle (moves left-to-right across sectors)
        obs_x = int(self.width * 0.50 + math.sin(t) * (self.width * 0.35))
        # Obstacle moves closer and further (y-position on floor)
        obs_y = int(floor_y + 40 + math.cos(t * 0.5) * 60)
        obs_w = 80
        obs_h = 100

        # Draw a synthetic cardboard box obstacle
        cv2.rectangle(frame, (obs_x - obs_w // 2, obs_y), (obs_x + obs_w // 2, obs_y + obs_h), (35, 75, 140), -1)
        cv2.rectangle(frame, (obs_x - obs_w // 2, obs_y), (obs_x + obs_w // 2, obs_y + obs_h), (255, 255, 255), 2)

        # Draw a simulated human stick figure in the background (to test YOLO / HOG)
        person_x = int(self.width * 0.50 - math.sin(t * 0.3) * 150)
        person_y = floor_y - 20
        # Head
        cv2.circle(frame, (person_x, person_y - 80), 18, (30, 30, 30), -1)
        # Torso
        cv2.line(frame, (person_x, person_y - 62), (person_x, person_y - 10), (30, 30, 30), 8)
        # Legs
        cv2.line(frame, (person_x, person_y - 10), (person_x - 15, person_y + 40), (30, 30, 30), 6)
        cv2.line(frame, (person_x, person_y - 10), (person_x + 15, person_y + 40), (30, 30, 30), 6)

        # HUD Annotation
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"MOCK SENSOR FEED | Simulated Scene (t={t:.1f}s)", (10, 25), font, 0.6, (0, 0, 200), 2)
        cv2.putText(frame, f"Simulated Obstacle X: {obs_x}px, Y: {obs_y}px", (10, 50), font, 0.5, (0, 0, 0), 1)

        # ----------------------------------------------------
        # 2. Publish ROS 2 Image Message
        # ----------------------------------------------------
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_optical_link'
        self.pub_image.publish(img_msg)

        # ----------------------------------------------------
        # 3. Publish Synthetic Ultrasonic Range
        # Oscillates between 0.25m and 1.80m
        # ----------------------------------------------------
        sim_dist = 1.0 + 0.75 * math.sin(t * 0.5)
        range_msg = Range()
        range_msg.header.stamp = img_msg.header.stamp
        range_msg.header.frame_id = 'ultrasonic_front_link'
        range_msg.radiation_type = Range.ULTRASONIC
        range_msg.min_range = 0.02
        range_msg.max_range = 3.00
        range_msg.field_of_view = 0.26
        range_msg.range = float(sim_dist)
        self.pub_range.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MockSensorPublisher stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
