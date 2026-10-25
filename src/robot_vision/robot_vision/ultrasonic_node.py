#!/usr/bin/env python3
"""
Ultrasonic Rangefinder Node for Raspberry Pi (Robot Host).
Reads HC-SR04 ultrasonic sensors via Raspberry Pi GPIO, applies median noise filtering,
and publishes distance data using:
  - sensor_msgs/msg/Range (standard ROS 2 perception message for RViz / Nav2)
  - geometry_msgs/msg/Vector3 on /ultrasonic_raw (convenience aggregate for APF navigator)

Hardware Note:
HC-SR04 ECHO pin outputs 5V. Use a voltage divider (e.g. 1kΩ and 2kΩ resistors)
to step down the ECHO signal to 3.3V before connecting to Raspberry Pi GPIO pins!
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3
import time
from collections import deque
from typing import Optional

# Graceful import of RPi.GPIO with mock fallback for testing off-robot
try:
    import RPi.GPIO as GPIO
    HAVE_RPI_GPIO = True
except (ImportError, RuntimeError):
    HAVE_RPI_GPIO = False


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # ----------------------------------------------------
        # Parameters (BCM Pin Numbering)
        # ----------------------------------------------------
        self.declare_parameter('trig_front', 23)
        self.declare_parameter('echo_front', 24)
        self.declare_parameter('enable_side_sensors', False)
        self.declare_parameter('trig_left', 17)
        self.declare_parameter('echo_left', 27)
        self.declare_parameter('trig_right', 5)
        self.declare_parameter('echo_right', 6)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('field_of_view_rad', 0.26)  # ~15 degrees
        self.declare_parameter('min_range_m', 0.02)        # 2 cm
        self.declare_parameter('max_range_m', 3.00)        # 3.0 m (practical HC-SR04 range)
        self.declare_parameter('median_filter_size', 3)

        self.trig_front = int(self.get_parameter('trig_front').value)
        self.echo_front = int(self.get_parameter('echo_front').value)
        self.enable_sides = bool(self.get_parameter('enable_side_sensors').value)
        self.trig_left = int(self.get_parameter('trig_left').value)
        self.echo_left = int(self.get_parameter('echo_left').value)
        self.trig_right = int(self.get_parameter('trig_right').value)
        self.echo_right = int(self.get_parameter('echo_right').value)
        self.fov = float(self.get_parameter('field_of_view_rad').value)
        self.min_range = float(self.get_parameter('min_range_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        # Sliding window queues for median filtering
        window_size = int(self.get_parameter('median_filter_size').value)
        self.filter_front = deque(maxlen=window_size)
        self.filter_left = deque(maxlen=window_size)
        self.filter_right = deque(maxlen=window_size)

        # ----------------------------------------------------
        # Hardware Setup
        # ----------------------------------------------------
        if HAVE_RPI_GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            GPIO.setup(self.trig_front, GPIO.OUT)
            GPIO.setup(self.echo_front, GPIO.IN)
            GPIO.output(self.trig_front, False)

            if self.enable_sides:
                GPIO.setup(self.trig_left, GPIO.OUT)
                GPIO.setup(self.echo_left, GPIO.IN)
                GPIO.output(self.trig_left, False)

                GPIO.setup(self.trig_right, GPIO.OUT)
                GPIO.setup(self.echo_right, GPIO.IN)
                GPIO.output(self.trig_right, False)

            self.get_logger().info(
                f"RPi.GPIO initialized. Front Sensor: TRIG={self.trig_front}, ECHO={self.echo_front} | Sides Enabled={self.enable_sides}"
            )
        else:
            self.get_logger().warn(
                "RPi.GPIO unavailable. UltrasonicNode running in SIMULATION/MOCK mode."
            )

        # Allow sensors to settle
        time.sleep(0.1)

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------
        # Standard ROS 2 Range topics for each sensor
        self.pub_range_front = self.create_publisher(Range, '/ultrasonic/front', 10)
        if self.enable_sides:
            self.pub_range_left = self.create_publisher(Range, '/ultrasonic/left', 10)
            self.pub_range_right = self.create_publisher(Range, '/ultrasonic/right', 10)

        # Convenience aggregate vector for quick consumption by navigator_node
        self.pub_vector = self.create_publisher(Vector3, '/ultrasonic_raw', 10)

        # Sampling timer loop
        timer_period = 1.0 / max(1.0, rate_hz)
        self.timer = self.create_timer(timer_period, self.sample_and_publish)

    def _read_distance_raw(self, trig_pin: int, echo_pin: int) -> Optional[float]:
        """
        Triggers an ultrasonic pulse and calculates distance in meters.
        Includes a 30ms guard timeout to prevent blocking on lost echo.
        """
        if not HAVE_RPI_GPIO:
            # Mock mode: return clean default 2.0 meters
            return 2.0

        # Send 10 microsecond trigger pulse
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)

        pulse_start = time.perf_counter()
        timeout_start = pulse_start

        # Wait for echo to go HIGH (max wait 0.03 seconds = ~5m)
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.perf_counter()
            if pulse_start - timeout_start > 0.030:
                return None

        # Wait for echo to go LOW (pulse duration)
        pulse_end = pulse_start
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.perf_counter()
            if pulse_end - pulse_start > 0.030:
                return None

        duration = pulse_end - pulse_start
        # Speed of sound in dry air at 20°C is ~343 m/s.
        # Distance = (duration * 343) / 2
        distance = (duration * 343.0) / 2.0
        return distance

    def _get_filtered_distance(self, trig_pin: int, echo_pin: int, history_deque: deque) -> float:
        raw_dist = self._read_distance_raw(trig_pin, echo_pin)

        if raw_dist is None or raw_dist > self.max_range:
            filtered_val = self.max_range
        elif raw_dist < self.min_range:
            filtered_val = self.min_range
        else:
            filtered_val = raw_dist

        history_deque.append(filtered_val)
        # Median filter to eliminate acoustic multipath spikes
        median_val = float(sorted(history_deque)[len(history_deque) // 2])
        return median_val

    def _build_range_msg(self, distance: float, frame_id: str) -> Range:
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASONIC
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = float(distance)
        return msg

    def sample_and_publish(self):
        # 1. Sample Front Sensor
        d_front = self._get_filtered_distance(self.trig_front, self.echo_front, self.filter_front)

        # 2. Sample Side Sensors if enabled
        if self.enable_sides:
            # Small delay between pings to prevent cross-echo interference
            time.sleep(0.005)
            d_left = self._get_filtered_distance(self.trig_left, self.echo_left, self.filter_left)
            time.sleep(0.005)
            d_right = self._get_filtered_distance(self.trig_right, self.echo_right, self.filter_right)
        else:
            d_left = self.max_range
            d_right = self.max_range

        # 3. Publish Standard ROS 2 Range Messages
        self.pub_range_front.publish(self._build_range_msg(d_front, 'ultrasonic_front_link'))

        if self.enable_sides:
            self.pub_range_left.publish(self._build_range_msg(d_left, 'ultrasonic_left_link'))
            self.pub_range_right.publish(self._build_range_msg(d_right, 'ultrasonic_right_link'))

        # 4. Publish Vector3 Aggregate for direct APF Navigation
        vec_msg = Vector3()
        vec_msg.x = float(d_front)
        vec_msg.y = float(d_left)
        vec_msg.z = float(d_right)
        self.pub_vector.publish(vec_msg)

    def destroy_node(self):
        if HAVE_RPI_GPIO:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("UltrasonicNode shut down cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
