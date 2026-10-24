import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

        # -----------------------------
        # ROS 2 Parameters (Tunable gains & thresholds)
        # -----------------------------
        self.declare_parameter('kp_linear', 0.5)          # Proportional gain for linear velocity
        self.declare_parameter('kp_angular', 1.5)         # Proportional gain for angular velocity
        self.declare_parameter('max_linear_vel', 0.3)     # Max linear speed (m/s)
        self.declare_parameter('max_angular_vel', 1.0)    # Max angular speed (rad/s)
        self.declare_parameter('goal_tolerance', 0.15)    # Distance threshold to consider arrived (meters)
        self.declare_parameter('turn_in_place_threshold', 0.35) # Angle error (~20 deg) above which robot turns in place

        # -----------------------------
        # 1. Listen to the ESP sensor data (The Eyes)
        # -----------------------------
        self.target_sub = self.create_subscription(
            Point,
            '/target_location',
            self.target_callback,
            10
        )

        # -----------------------------
        # 2. Track Robot Position (The Odometry / Pose)
        # -----------------------------
        # Standard ROS 2 wheel odometry topic
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Optional: ESP-based robot tracker fallback (if teammate publishes robot's global Point)
        self.robot_marker_sub = self.create_subscription(
            Point,
            '/robot_location',
            self.robot_location_callback,
            10
        )

        # -----------------------------
        # 3. Send commands to the robot motors (The Body)
        # -----------------------------
        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # State Variables
        self.target_x = None
        self.target_y = None
        self.has_target = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0   # Facing angle in radians [-pi, pi]
        self.has_odom = False

        # -----------------------------
        # 4. Control loop running at 10Hz (0.1s interval)
        # -----------------------------
        self.timer = self.create_timer(0.1, self.navigate_to_target)
        self.get_logger().info("NavigatorNode initialized. Awaiting target & odometry...")

    def target_callback(self, msg: Point):
        """Update target coordinates published by the through-wall ESP array."""
        self.target_x = msg.x
        self.target_y = msg.y
        self.has_target = True
        self.get_logger().info(
            f"New target received -> X: {self.target_x:.2f}, Y: {self.target_y:.2f}"
        )

    def odom_callback(self, msg: Odometry):
        """Extract (X, Y) and convert orientation quaternion to yaw (theta)."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Quaternion to Euler yaw (Z-axis rotation)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny_cosp, cosy_cosp)

        self.has_odom = True

    def robot_location_callback(self, msg: Point):
        """Fallback callback if an on-board ESP publishes robot position directly."""
        self.robot_x = msg.x
        self.robot_y = msg.y
        # If the point message provides orientation in z (in radians), use it:
        if abs(msg.z) > 0.0001:
            self.robot_theta = msg.z
        self.has_odom = True

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi] to handle angle wrap-around.
        Prevents robot from spinning 350 degrees when crossing the +/- pi boundary.
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def navigate_to_target(self):
        """10Hz Navigation control loop."""
        # 1. Sanity check: Ensure we have both a target and current robot location
        if not self.has_target:
            return

        if not self.has_odom:
            self.get_logger().info(
                "Waiting for robot odometry (/odom or /robot_location)...",
                throttle_duration_sec=3.0
            )
            return

        # Fetch current dynamic parameter values
        kp_lin = self.get_parameter('kp_linear').value
        kp_ang = self.get_parameter('kp_angular').value
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value
        tolerance = self.get_parameter('goal_tolerance').value
        turn_thresh = self.get_parameter('turn_in_place_threshold').value

        # 2. Calculate Distance to target
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance = math.hypot(dx, dy)

        msg = Twist()

        # 3. Check if target reached
        if distance <= tolerance:
            # Reached target: stop robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            self.get_logger().info(
                f"Target reached! Distance: {distance:.2f}m <= {tolerance:.2f}m",
                throttle_duration_sec=2.0
            )
            return

        # 4. Calculate desired angle to target & heading error
        desired_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(desired_theta - self.robot_theta)

        # 5. Differential Drive Controller Strategy
        if abs(angle_error) > turn_thresh:
            # Significant angle mismatch: rotate in place first before driving forward.
            # This prevents wide drift arcs that collide with maze walls.
            msg.linear.x = 0.0
            raw_angular = kp_ang * angle_error
            # Clamp angular speed
            msg.angular.z = max(min(raw_angular, max_ang), -max_ang)
        else:
            # Facing roughly toward target: drive forward while maintaining heading.
            # Scale linear speed with distance (P-control) and cosine of angle error
            raw_linear = kp_lin * distance * max(0.0, math.cos(angle_error))
            msg.linear.x = max(min(raw_linear, max_lin), 0.0)

            raw_angular = kp_ang * angle_error
            msg.angular.z = max(min(raw_angular, max_ang), -max_ang)

        # 6. Publish command to motor node
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()