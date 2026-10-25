import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import math

class NavigatorNode(Node):
    """
    Artificial Potential Field (APF) Navigator Node for Search-and-Rescue Robot.
    
    Combines:
      - Attractive Force (Pull): Scales with distance to the through-wall ESP target.
      - Repulsive Force (Push): Inverse-square repulsion from obstacles within the danger zone.
      - Boredom Timer: Solves APF local minima / U-trap paralysis with an escape maneuver.
    """
    def __init__(self):
        super().__init__('navigator_node')

        # -----------------------------
        # ROS 2 Parameters (Tunable in real-time)
        # -----------------------------
        self.declare_parameter('k_att', 0.40)                    # Attraction scaling factor
        self.declare_parameter('k_rep', 1.20)                    # Canal bank repulsion scaling factor
        self.declare_parameter('danger_zone', 0.40)              # Doubled danger zone: 40 cm
        self.declare_parameter('max_repulsion_force', 3.0)       # Maximum repulsion clamp
        self.declare_parameter('max_linear_vel', 0.015)          # Ultra-calm smooth crawl (0.015 m/s)
        self.declare_parameter('max_angular_vel', 0.028)         # Gentle, controlled rotation speed (0.028 rad/s)
        self.declare_parameter('goal_tolerance', 0.20)           # Target arrival radius (meters)
        self.declare_parameter('kp_angular', 0.40)               # Soft proportional steering towards resultant
        self.declare_parameter('boredom_threshold_speed', 0.01)  # Speed below which robot is considered stalled
        self.declare_parameter('boredom_trigger_time', 15.0)     # Seconds stalled before triggering escape
        self.declare_parameter('escape_duration', 0.5)           # Seconds to execute escape turn

        # -----------------------------
        # Subscriptions
        # -----------------------------
        # 1. Target from ESP array (The Eyes)
        self.target_sub = self.create_subscription(
            Point,
            '/target_location',
            self.target_callback,
            10
        )

        # 2. Robot Localization (/odom or /robot_location)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.robot_location_sub = self.create_subscription(
            Point,
            '/robot_location',
            self.robot_location_callback,
            10
        )

        # 3. Obstacle Perception (from camera or ultrasonic sensors)
        # Vector3: x = front_distance, y = left_distance, z = right_distance (in meters)
        self.obstacle_sub = self.create_subscription(
            Vector3,
            '/obstacle_status',
            self.obstacle_callback,
            10
        )

        # 4. Detected Victim/Person from Camera (The Eyes)
        self.victim_sub = self.create_subscription(
            Point,
            '/detected_person',
            self.victim_callback,
            10
        )

        # 5. Direct Ultrasonic Sensor Subscriptions (center, left, right)
        self.sub_us_c1 = self.create_subscription(Float32, '/ultrasonic_center', self._us_center_cb, 10)
        self.sub_us_c2 = self.create_subscription(Float32, 'ultrasonic_center', self._us_center_cb, 10)
        self.sub_us_l1 = self.create_subscription(Float32, '/ultrasonic_left', self._us_left_cb, 10)
        self.sub_us_l2 = self.create_subscription(Float32, 'ultrasonic_left', self._us_left_cb, 10)
        self.sub_us_r1 = self.create_subscription(Float32, '/ultrasonic_right', self._us_right_cb, 10)
        self.sub_us_r2 = self.create_subscription(Float32, 'ultrasonic_right', self._us_right_cb, 10)

        # -----------------------------
        # Publisher (The Body)
        # -----------------------------
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # -----------------------------
        # Internal State
        # -----------------------------
        # Initial State: Autonomous Exploration mode (starts moving immediately forward and navigates around walls)
        self.target_x = 2.0
        self.target_y = 0.0
        self.has_target = True
        self.has_odom = True
        self.using_internal_odom = True
        self.mode = "EXPLORATION"  # "EXPLORATION", "TARGET_PURSUIT", or "VICTIM_PURSUIT"

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # Heading in radians [-pi, pi]

        # Sensor readings (default to 2.0 meters = clear)
        self.front_dist = 2.0
        self.left_dist = 2.0
        self.right_dist = 2.0

        # Dedicated 90-Degree Wall Avoidance Controller (Strictly caps turn to 1/4 of 360: ~90 deg)
        self.is_avoiding_wall = False
        self.avoid_turn_dir = 1.0
        self.avoid_turn_time = 0.0
        self.avoid_cooldown = 0.0
        self.stalled_time = 0.0

        # Exploration Memory
        self.visited_grid = {}        # (cell_x, cell_y) -> visit count

        # 10Hz Control Loop (0.1s interval)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("APF Navigator initialized in EXPLORATION (v=0.015 m/s, ultra-calm 50cm buffer)...")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def victim_callback(self, msg: Point):
        # msg: x=dist_meters, y=bearing_deg, z=confidence
        # Only pursue victim if explicitly in VICTIM_PURSUIT mode and confidence is high
        if self.mode == "VICTIM_PURSUIT" and msg.z >= 0.70 and msg.x > 0.3:
            bearing_rad = math.radians(msg.y)
            abs_theta = self.robot_theta + bearing_rad
            self.target_x = self.robot_x + msg.x * math.cos(abs_theta)
            self.target_y = self.robot_y + msg.x * math.sin(abs_theta)
            self.has_target = True
            self.get_logger().info(
                f"Victim sighted at {msg.x:.2f}m ({msg.y:.1f} deg)! Navigating to target ({self.target_x:.2f}, {self.target_y:.2f})"
            )

    def target_callback(self, msg: Point):
        # Only log if target actually changes by more than 5cm
        if not self.has_target or math.hypot(msg.x - self.target_x, msg.y - self.target_y) > 0.05:
            self.target_x = msg.x
            self.target_y = msg.y
            self.has_target = True
            self.mode = "TARGET_PURSUIT"
            self.get_logger().info(
                f"New target -> ({self.target_x:.2f}, {self.target_y:.2f}) [TARGET_PURSUIT]"
            )

    def odom_callback(self, msg: Odometry):
        self.using_internal_odom = False
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        # Auto-detect simulation restart / robot teleport (>0.8m sudden jump)
        if self.has_odom:
            if math.hypot(new_x - self.robot_x, new_y - self.robot_y) > 0.80:
                self.get_logger().info("Simulation restart / robot teleport detected! Resetting navigation memory.")
                self.visited_grid.clear()
                self.stalled_time = 0.0
                self.is_avoiding_wall = False
                self.avoid_turn_time = 0.0

        self.robot_x = new_x
        self.robot_y = new_y

        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def robot_location_callback(self, msg: Point):
        self.robot_x = msg.x
        self.robot_y = msg.y
        if abs(msg.z) > 0.0001:
            self.robot_theta = msg.z
        self.has_odom = True

    def obstacle_callback(self, msg: Vector3):
        # Fast attack (immediate reaction when obstacle approaches), slow release (filter clearing)
        for attr, val in (('front_dist', msg.x), ('left_dist', msg.y), ('right_dist', msg.z)):
            curr = getattr(self, attr)
            if val < curr:
                # Danger closer: update instantly (85% new reading)
                setattr(self, attr, 0.85 * val + 0.15 * curr)
            else:
                # Path clearing: smooth with 35% weight
                setattr(self, attr, 0.35 * val + 0.65 * curr)

    def _us_center_cb(self, msg: Float32):
        if msg.data > 0.0:
            m = float(msg.data) / 100.0 if float(msg.data) > 5.0 else float(msg.data)
            if m < self.front_dist:
                self.front_dist = 0.85 * m + 0.15 * self.front_dist
            else:
                self.front_dist = 0.35 * m + 0.65 * self.front_dist

    def _us_left_cb(self, msg: Float32):
        if msg.data > 0.0:
            m = float(msg.data) / 100.0 if float(msg.data) > 5.0 else float(msg.data)
            if m < self.left_dist:
                self.left_dist = 0.85 * m + 0.15 * self.left_dist
            else:
                self.left_dist = 0.35 * m + 0.65 * self.left_dist

    def _us_right_cb(self, msg: Float32):
        if msg.data > 0.0:
            m = float(msg.data) / 100.0 if float(msg.data) > 5.0 else float(msg.data)
            if m < self.right_dist:
                self.right_dist = 0.85 * m + 0.15 * self.right_dist
            else:
                self.right_dist = 0.35 * m + 0.65 * self.right_dist

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    # -----------------------------
    # Main APF Control Loop (10Hz)
    # -----------------------------
    def control_loop(self):
        if not self.has_target or not self.has_odom:
            # In STANDBY: actively hold motors at zero
            cmd = Twist()
            self.vel_pub.publish(cmd)
            return

        cmd = Twist()

        # Distance to goal
        dx_world = self.target_x - self.robot_x
        dy_world = self.target_y - self.robot_y
        dist_to_goal = math.hypot(dx_world, dy_world)

        goal_tol = self.get_parameter('goal_tolerance').value
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value

        # Record exploration visit trail (0.25m bins)
        cell = (round(self.robot_x * 4.0) / 4.0, round(self.robot_y * 4.0) / 4.0)
        self.visited_grid[cell] = self.visited_grid.get(cell, 0) + 1

        # Decrement turn cooldown
        if self.avoid_cooldown > 0.0:
            self.avoid_cooldown -= self.dt

        # -----------------------------
        # 1. Goal Arrival Check
        # -----------------------------
        if dist_to_goal <= goal_tol:
            if self.mode == "EXPLORATION":
                # In exploration mode, extend target 2.0m forward in current heading
                self.target_x = self.robot_x + 2.0 * math.cos(self.robot_theta)
                self.target_y = self.robot_y + 2.0 * math.sin(self.robot_theta)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.vel_pub.publish(cmd)
                self.has_target = False
                self.mode = "STANDBY"
                self.get_logger().info(
                    f"Target reached! (Dist: {dist_to_goal:.2f}m <= {goal_tol:.2f}m). Standing by.",
                    throttle_duration_sec=2.0
                )
                self.stalled_time = 0.0
                return

        # -----------------------------
        # 2. Strict Wall Avoidance Controller
        # Pure in-place rotation strictly capped to half of previous turn (~0.11s)
        # -----------------------------
        if self.is_avoiding_wall:
            turn_rate = max_ang  # 0.028 rad/s
            self.avoid_turn_time += self.dt

            # Turn duration halved to 0.11s for exactly half the turn angle (~10 deg nudge)
            if self.avoid_turn_time >= 0.11 or self.front_dist >= 0.55:
                self.is_avoiding_wall = False
                self.avoid_turn_time = 0.0
                self.avoid_cooldown = 2.0  # 2.0s cooldown so it drives smoothly forward
                if self.using_internal_odom:
                    self.robot_theta = self.normalize_angle(self.robot_theta + self.avoid_turn_dir * 0.175)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.vel_pub.publish(cmd)
                self.get_logger().info(
                    f"Completed exact half-turn in place! Front={self.front_dist:.2f}m. Resuming straight forward crawl."
                )
                if self.mode == "EXPLORATION":
                    # Reorient forward exploration target 2.0m along new heading
                    self.target_x = self.robot_x + 2.0 * math.cos(self.robot_theta)
                    self.target_y = self.robot_y + 2.0 * math.sin(self.robot_theta)
                return
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.avoid_turn_dir * turn_rate
                self.vel_pub.publish(cmd)
                return

        # Trigger controlled turn when front clearance reaches 50 cm (doubled danger distance)
        if self.front_dist <= 0.50 and self.avoid_cooldown <= 0.0:
            self.is_avoiding_wall = True
            self.avoid_turn_time = 0.0
            # Choose the side with more room (Left if left_dist >= right_dist, else Right)
            self.avoid_turn_dir = 1.0 if self.left_dist >= self.right_dist else -1.0
            self.get_logger().warn(
                f"Obstacle ahead ({self.front_dist:.2f}m <= 0.50m danger buffer)! Executing smooth in-place nudge {'LEFT' if self.avoid_turn_dir > 0 else 'RIGHT'}..."
            )
            cmd.linear.x = 0.0
            cmd.angular.z = self.avoid_turn_dir * max_ang
            self.vel_pub.publish(cmd)
            return

        # -----------------------------
        # 3. Calculate Attraction Vector (The Pull)
        # Formula: F_att = K_att * (Target_xy - Robot_xy)
        # -----------------------------
        k_att = self.get_parameter('k_att').value
        att_mag = min(k_att * dist_to_goal, 0.80)
        f_att_world_x = att_mag * (dx_world / max(dist_to_goal, 0.001))
        f_att_world_y = att_mag * (dy_world / max(dist_to_goal, 0.001))

        # Transform Attraction Vector into the Robot's Local Frame
        cos_t = math.cos(self.robot_theta)
        sin_t = math.sin(self.robot_theta)
        f_att_robot_x = f_att_world_x * cos_t + f_att_world_y * sin_t
        f_att_robot_y = -f_att_world_x * sin_t + f_att_world_y * cos_t

        # -----------------------------
        # 4. Calculate Side Repulsions (Doubled to 50cm Buffer)
        # -----------------------------
        k_rep = self.get_parameter('k_rep').value

        f_rep_left = 0.0
        if self.left_dist < 0.50:
            f_rep_left = k_rep * max(0.0, (0.50 - self.left_dist) / 0.30)

        f_rep_right = 0.0
        if self.right_dist < 0.50:
            f_rep_right = k_rep * max(0.0, (0.50 - self.right_dist) / 0.30)

        # -----------------------------
        # 5. Total Resultant Vector
        # -----------------------------
        f_total_x = max(0.08, f_att_robot_x)
        f_total_y = f_att_robot_y - f_rep_left + f_rep_right

        resultant_magnitude = math.hypot(f_total_x, f_total_y)
        heading_error = math.atan2(f_total_y, f_total_x)  # Heading error in robot frame
        heading_error = max(-math.pi / 3.0, min(math.pi / 3.0, heading_error))

        # -----------------------------
        # 6. Decoupled Motor Commands
        # In EXPLORATION: drive strictly straight forward (cmd.angular.z = 0.0).
        # Rotation ONLY happens when an obstacle is detected ahead!
        # -----------------------------
        if self.mode == "EXPLORATION":
            # Pure straight forward crawl: absolutely zero rotation when path ahead is clear
            front_slowdown = min(1.0, max(0.0, (self.front_dist - 0.45) / 0.35))
            cmd.linear.x = max_lin * front_slowdown
            cmd.angular.z = 0.0
        else:
            # Target Pursuit: steer towards specific coordinate target with generous 20-deg deadband
            heading_threshold = 0.35  # ~20 degrees
            if abs(heading_error) > heading_threshold:
                cmd.linear.x = 0.0
                turn_sign = 1.0 if heading_error > 0 else -1.0
                cmd.angular.z = turn_sign * max_ang
            else:
                front_slowdown = min(1.0, max(0.0, (self.front_dist - 0.45) / 0.35))
                cmd.linear.x = max_lin * front_slowdown
                cmd.angular.z = 0.0

        # Internal dead-reckoning integration if external odometry is not available
        if self.using_internal_odom:
            self.robot_theta = self.normalize_angle(self.robot_theta + cmd.angular.z * self.dt)
            self.robot_x += cmd.linear.x * math.cos(self.robot_theta) * self.dt
            self.robot_y += cmd.linear.x * math.sin(self.robot_theta) * self.dt

        self.get_logger().info(
            f"APF [{self.mode}]: TargetDist={dist_to_goal:.2f}m | Front={self.front_dist:.2f}m, L={self.left_dist:.2f}m, R={self.right_dist:.2f}m -> CmdVel: v={cmd.linear.x:.2f} m/s, w={cmd.angular.z:.2f} rad/s",
            throttle_duration_sec=1.0
        )
        self.vel_pub.publish(cmd)

        # -----------------------------
        # 7. Check for Local Minima Trap (The Boredom Timer)
        # -----------------------------
        boredom_speed_thresh = self.get_parameter('boredom_threshold_speed').value
        boredom_trigger_time = self.get_parameter('boredom_trigger_time').value

        is_stalled = (cmd.linear.x < boredom_speed_thresh) or (self.front_dist < 0.22 and cmd.linear.x < 0.05)
        if dist_to_goal > 0.8 and is_stalled:
            self.stalled_time += self.dt
            if self.stalled_time >= boredom_trigger_time:
                self.get_logger().warn(
                    f"Local minimum detected! Stalled for {self.stalled_time:.1f}s. Initiating strict 90-deg turn!"
                )
                self.is_avoiding_wall = True
                self.avoid_turn_time = 0.0
                self.avoid_turn_dir = 1.0 if self.left_dist >= self.right_dist else -1.0
                self.stalled_time = 0.0
                return
        else:
            self.stalled_time = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
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