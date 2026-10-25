import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Unexport any stale sysfs pins to prevent lgpio 'GPIO busy' kernel collisions
import os
import time
for pin in (23, 24, 17, 27, 5, 6):
    try:
        if os.path.exists(f'/sys/class/gpio/gpio{pin}'):
            with open('/sys/class/gpio/unexport', 'w') as f:
                f.write(str(pin))
    except Exception:
        pass

# Free any lingering lgpio lines if handle was kept open
try:
    import lgpio
    _h = lgpio.gpiochip_open(0)
    for _p in (23, 24, 17, 27, 5, 6):
        try:
            lgpio.gpio_free(_h, _p)
        except Exception:
            pass
    lgpio.gpiochip_close(_h)
except Exception:
    pass

from gpiozero import DistanceSensor


class MultiUltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('multi_ultrasonic_publisher')
        
        # Create three separate publishers
        self.pub_center = self.create_publisher(Float32, 'ultrasonic_center', 10)
        self.pub_left = self.create_publisher(Float32, 'ultrasonic_left', 10)
        self.pub_right = self.create_publisher(Float32, 'ultrasonic_right', 10)
        
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        # Initialize sensors with per-sensor retry resilience
        self.sensor_center = None
        self.sensor_left = None
        self.sensor_right = None

        for attempt in range(3):
            if self.sensor_center is None:
                try:
                    self.sensor_center = DistanceSensor(echo=24, trigger=23)
                    self.get_logger().info('Center Ultrasonic initialized (23/24)')
                except Exception as e:
                    if attempt == 2:
                        self.get_logger().warn(f'Center Ultrasonic init: {e}')
            if self.sensor_left is None:
                try:
                    self.sensor_left = DistanceSensor(echo=27, trigger=17)
                    self.get_logger().info('Left Ultrasonic initialized (17/27)')
                except Exception as e:
                    if attempt == 2:
                        self.get_logger().warn(f'Left Ultrasonic init: {e}')
            if self.sensor_right is None:
                try:
                    self.sensor_right = DistanceSensor(echo=6, trigger=5)
                    self.get_logger().info('Right Ultrasonic initialized (5/6)')
                except Exception as e:
                    if attempt == 2:
                        self.get_logger().warn(f'Right Ultrasonic init: {e}')
            if self.sensor_center and self.sensor_left and self.sensor_right:
                break
            time.sleep(0.2)

    def timer_callback(self):
        if self.sensor_center:
            try:
                msg_c = Float32(data=float(self.sensor_center.distance * 100.0))
                self.pub_center.publish(msg_c)
            except Exception:
                pass

        if self.sensor_left:
            try:
                msg_l = Float32(data=float(self.sensor_left.distance * 100.0))
                self.pub_left.publish(msg_l)
            except Exception:
                pass

        if self.sensor_right:
            try:
                msg_r = Float32(data=float(self.sensor_right.distance * 100.0))
                self.pub_right.publish(msg_r)
            except Exception:
                pass

    def destroy_node(self):
        for s in [self.sensor_center, self.sensor_left, self.sensor_right]:
            if s is not None:
                try:
                    s.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiUltrasonicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()