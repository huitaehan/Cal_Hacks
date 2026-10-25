#!/usr/bin/env python3
"""
Single-Process Unified Robot Bringup for Raspberry Pi (Robot Host).
Runs both:
  1. PiCameraNode: Logitech Brio JPEG compressed stream (/image_raw/compressed) @ 20 FPS
  2. MultiUltrasonicPublisher: Triple HC-SR04 distance sensors (/ultrasonic_center/left/right)

Features:
  - Clean startup: auto-releases stale GPIO pins and camera device memory.
  - MultiThreadedExecutor: runs both nodes concurrently with zero thread contention.
  - Clean shutdown: releases all hardware cleanly on Ctrl+C.
"""

import sys
import time
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
import cv2

# Hardware pin cleanup to prevent lgpio 'GPIO busy' locks
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.cleanup([23, 24, 17, 27, 5, 6])
except Exception:
    pass

try:
    from gpiozero.pins.rpigpio import RPiGPIOFactory
    from gpiozero import Device
    Device.pin_factory = RPiGPIOFactory()
except Exception:
    pass

from gpiozero import DistanceSensor


class UnifiedPiCamera(Node):
    def __init__(self):
        super().__init__('pi_cam_node')
        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('jpeg_quality', 60)

        dev = self.get_parameter('video_device').value
        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        idx = int(dev.replace('/dev/video', '')) if isinstance(dev, str) and '/dev/video' in dev else int(dev)
        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(idx)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera on /dev/video{idx}!")
            self.cap = None
            return

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Camera opened: {actual_w}x{actual_h} @ {fps} FPS -> Publishing '/image_raw/compressed'")

        self.pub_compressed = self.create_publisher(CompressedImage, '/image_raw/compressed', qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max(1.0, fps), self.capture_and_publish)

    def capture_and_publish(self):
        if not self.cap or not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        success, enc_buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if success:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_optical_link'
            msg.format = 'jpeg'
            msg.data = enc_buf.tobytes()
            self.pub_compressed.publish(msg)

    def close(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()


class UnifiedUltrasonic(Node):
    def __init__(self):
        super().__init__('multi_ultrasonic_publisher')
        self.pub_center = self.create_publisher(Float32, '/ultrasonic_center', 10)
        self.pub_left = self.create_publisher(Float32, '/ultrasonic_left', 10)
        self.pub_right = self.create_publisher(Float32, '/ultrasonic_right', 10)

        # Initialize sensors
        self.sensor_center = DistanceSensor(echo=24, trigger=23)
        self.sensor_left = DistanceSensor(echo=27, trigger=17)
        self.sensor_right = DistanceSensor(echo=6, trigger=5)

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info("Triple Ultrasonic Sensors initialized (Center: 23/24, Left: 17/27, Right: 5/6)")

    def timer_callback(self):
        try:
            msg_c = Float32(data=float(self.sensor_center.distance * 100.0))
            self.pub_center.publish(msg_c)
        except Exception:
            pass
        try:
            msg_l = Float32(data=float(self.sensor_left.distance * 100.0))
            self.pub_left.publish(msg_l)
        except Exception:
            pass
        try:
            msg_r = Float32(data=float(self.sensor_right.distance * 100.0))
            self.pub_right.publish(msg_r)
        except Exception:
            pass

    def close(self):
        for s in [self.sensor_center, self.sensor_left, self.sensor_right]:
            try:
                s.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)

    cam_node = UnifiedPiCamera()
    us_node = UnifiedUltrasonic()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(cam_node)
    executor.add_node(us_node)

    print("\n========================================================")
    print("  ROBOT HOST ACTIVE: Streaming Camera & 3x Ultrasonic  ")
    print("  Press Ctrl+C to stop cleanly.                         ")
    print("========================================================\n")

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nStopping Raspberry Pi Robot Bringup...")
    finally:
        cam_node.close()
        us_node.close()
        cam_node.destroy_node()
        us_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            import RPi.GPIO as GPIO
            GPIO.cleanup()
        except Exception:
            pass
        print("Hardware released cleanly. Goodbye.")


if __name__ == '__main__':
    main()
