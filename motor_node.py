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

MAX_POWER = 40
MIN_POWER_CUTOFF = 12  # Minimum power to overcome DC motor static friction

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

    # Static friction deadband compensation: if non-zero, boost to minimum moving power
    if 0 < abs(value) < MIN_POWER_CUTOFF:
        value = MIN_POWER_CUTOFF if value > 0 else -MIN_POWER_CUTOFF

    forward, reverse = motors[name]
    duty = abs(value) / MAX_POWER

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

        # Invert linear and angular velocities to match physical robot orientation
        vx = -msg.linear.x
        vy = -msg.linear.y
        w = -msg.angular.z  # Positive angular.z turns left (CCW)

        # Max linear speed from APF navigator is ~0.25 m/s. Normalize to [0.0, 1.0]
        lin_speed = math.hypot(vx, vy)
        norm_linear = min(1.0, lin_speed / 0.25)
        linear_power = norm_linear * MAX_POWER

        # Normalize angular speed (~1.4 rad/s max) to [0.0, 1.0]
        norm_angular = max(-1.0, min(1.0, w / 1.40))
        angular_power = norm_angular * MAX_POWER

        # If both linear and angular commands are practically zero, stop
        if lin_speed < 0.02 and abs(w) < 0.05:
            stop_motors()
            return

        # Calculate translation heading angle
        alpha = math.atan2(vy, vx) if lin_speed >= 0.02 else 0.0
        ax = linear_power * math.cos(alpha)
        ay = linear_power * math.sin(alpha)

        # 3-wheel omni-directional kinematics (120-degree motor separation)
        # w adds equally to all 3 wheels for pure rotation
        w_term = 0.333 * angular_power
        f1 = 0.578 * ax - 0.333 * ay + w_term
        f2 = 0.567 * ay + w_term
        f3 = -0.578 * ax - 0.333 * ay + w_term

        # Scale proportionally to fit within target power without clipping
        target_power = max(linear_power, abs(angular_power))
        target_power = max(MIN_POWER_CUTOFF, min(MAX_POWER, target_power))

        max_f = max(abs(f1), abs(f2), abs(f3))
        if max_f > 0:
            f1 = int(f1 * target_power / max_f)
            f2 = int(f2 * target_power / max_f)
            f3 = int(f3 * target_power / max_f)
        else:
            f1, f2, f3 = 0, 0, 0

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
