#!/usr/bin/env python3
"""
ROS 2 3-Wheel Omni-Directional Motor Driver Node for Raspberry Pi.
Subscribes to '/cmd_vel' (geometry_msgs/msg/Twist) and drives 3 DC motors via
L298N / H-Bridge PWM (gpiozero).

Kinematics: 3-wheel Kiwi / omni-drive (120-degree wheel separation).
Includes:
  - Deadband compensation (overcomes DC motor static friction).
  - Safe rotation in place (fixes pure-yaw zero-multiplication bug).
  - 0.35s safety watchdog timer (auto-stops if Wi-Fi drops).
  - Clean GPIO release on shutdown.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice

# ---------- Motor GPIO Pin Setup ----------
# Physical BCM pins for Motor A, B, C (forward_pin, reverse_pin)
MOTOR_PINS = {
    "A": (17, 27),
    "B": (22, 23),
    "C": (24, 25),
}

MAX_POWER = 100
MAX_DUTY_LIMIT = 0.25   # Halved speed ceiling: 25% PWM duty cycle (soft, ultra-calm crawl)
MIN_DUTY_LIMIT = 0.18   # Lower limit: 18% PWM duty cycle (gentle breakaway torque)

# Activate Raspberry Pi internal pull-down resistors so pins NEVER float to HIGH
import subprocess
for _p in (17, 27, 22, 23, 24, 25):
    try:
        subprocess.run(["pinctrl", "set", str(_p), "pd"], capture_output=True)
    except Exception:
        pass
    try:
        subprocess.run(["raspi-gpio", "set", str(_p), "pd"], capture_output=True)
    except Exception:
        pass

motors = {}
try:
    motors = {
        name: (
            PWMOutputDevice(forward_pin, frequency=1000, initial_value=0.0),
            PWMOutputDevice(reverse_pin, frequency=1000, initial_value=0.0),
        )
        for name, (forward_pin, reverse_pin) in MOTOR_PINS.items()
    }
except Exception as e:
    print(f"[WARN] Failed to initialize hardware PWM devices: {e}")


def stop_motors():
    for forward, reverse in motors.values():
        try:
            forward.value = 0
            reverse.value = 0
        except Exception:
            pass


def set_motor(name, value):
    if name not in motors:
        return
    value = max(-MAX_POWER, min(MAX_POWER, int(value)))

    if abs(value) < 1:
        duty = 0.0
    else:
        # Scale value [1..MAX_POWER] smoothly to [MIN_DUTY_LIMIT..MAX_DUTY_LIMIT]
        fraction = min(1.0, abs(value) / float(MAX_POWER))
        duty = MIN_DUTY_LIMIT + fraction * (MAX_DUTY_LIMIT - MIN_DUTY_LIMIT)
        duty = min(MAX_DUTY_LIMIT, max(MIN_DUTY_LIMIT, duty))

    forward, reverse = motors[name]

    try:
        if value > 0:
            reverse.value = 0
            forward.value = duty
        elif value < 0:
            forward.value = 0
            reverse.value = duty
        else:
            forward.value = 0
            reverse.value = 0
    except Exception:
        pass


class OmniMotorNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Subscribe to standard ROS 2 velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Safety timeout: Stop motors if no command received in 0.35s
        self.timeout_sec = 0.35
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.check_timeout)

        self.get_logger().info("ROS 2 3-Wheel Omni Motor Driver Active. Listening on '/cmd_vel'...")

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        # Flip 180 degrees from previous setting:
        vx = msg.linear.x
        vy = msg.linear.y
        w = -msg.angular.z  # Positive angular.z turns left (CCW)

        # Normalize linear speed to gentle 0.10 m/s scale
        lin_speed = math.hypot(vx, vy)
        norm_linear = min(1.0, lin_speed / 0.10)
        linear_power = norm_linear * MAX_POWER

        # Normalize angular speed to gentle 0.35 rad/s scale
        norm_angular = max(-1.0, min(1.0, w / 0.35))
        angular_power = norm_angular * MAX_POWER

        # If both linear and angular commands are practically zero, stop
        if lin_speed < 0.005 and abs(w) < 0.005:
            stop_motors()
            return

        # Calculate translation heading angle
        alpha = math.atan2(vy, vx) if lin_speed >= 0.01 else 0.0
        ax = linear_power * math.cos(alpha)
        ay = linear_power * math.sin(alpha)

        # 3-wheel omni-directional kinematics (120-degree motor separation)
        w_term = 0.333 * angular_power
        f1 = 0.578 * ax - 0.333 * ay + w_term
        f2 = 0.567 * ay + w_term
        f3 = -0.578 * ax - 0.333 * ay + w_term

        # Scale proportionally to fit within target power without clipping
        target_power = max(linear_power, abs(angular_power))
        target_power = max(10.0, min(float(MAX_POWER), target_power))

        max_f = max(abs(f1), abs(f2), abs(f3))
        if max_f > 0:
            f1 = int(f1 * target_power / max_f)
            f2 = int(f2 * target_power / max_f)
            f3 = int(f3 * target_power / max_f)
        else:
            f1, f2, f3 = 0, 0, 0

        self.get_logger().info(
            f"Cmd: lin.x={msg.linear.x:.2f}, ang.z={msg.angular.z:.2f} -> Powers: A={f1:+3d}, B={f3:+3d}, C={f2:+3d}",
            throttle_duration_sec=1.0
        )

        set_motor("A", f1)
        set_motor("B", f3)
        set_motor("C", f2)

    def check_timeout(self):
        # Auto-stop if Wi-Fi drops or APF node crashes
        time_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_last_cmd > self.timeout_sec:
            stop_motors()


def main(args=None):
    rclpy.init(args=args)
    node = OmniMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping motors...")
    finally:
        stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for _p in (17, 27, 22, 23, 24, 25):
            try:
                subprocess.run(["pinctrl", "set", str(_p), "pd"], capture_output=True)
            except Exception:
                pass
            try:
                subprocess.run(["raspi-gpio", "set", str(_p), "pd"], capture_output=True)
            except Exception:
                pass


if __name__ == '__main__':
    main()
