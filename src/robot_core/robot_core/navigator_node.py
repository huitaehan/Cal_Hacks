import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
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
        self.declare_parameter('k_att', 0.5)                     # Attraction scaling factor
        self.declare_parameter('k_rep', 2.20)                    # Canal bank repulsion scaling factor
        self.declare_parameter('danger_zone', 0.55)              # Canal embankment width (meters)
        self.declare_parameter('max_repulsion_force', 8.0)       # Maximum repulsion clamp
        self.declare_parameter('max_linear_vel', 0.25)           # Max forward speed (m/s)
        self.declare_parameter('max_angular_vel', 1.40)          # Max rotation speed (rad/s)
        self.declare_parameter('goal_tolerance', 0.15)           # Target arrival radius (meters)
        self.declare_parameter('kp_angular', 2.0)                # Proportional gain for steering towards resultant
        self.declare_parameter('boredom_threshold_speed', 0.05)  # Speed below which robot is considered stalled
        self.declare_parameter('boredom_trigger_time', 2.5)      # Seconds stalled before triggering escape
        self.declare_parameter('escape_duration', 1.5)           # Seconds to execute escape turn

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

        # -----------------------------
        # Publisher (The Body)
        # -----------------------------
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # -----------------------------
        # Internal State
        # -----------------------------
        self.target_x = 0.0
        self.target_y = 0.0
        self.has_target = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # Heading in radians [-pi, pi]
        self.has_odom = False

        # Sensor readings (default to 2.0 meters = clear)
        self.front_dist = 2.0
        self.left_dist = 2.0
        self.right_dist = 2.0

        # Boredom & Local Minima Escape State
        self.stalled_time = 0.0
        self.is_escaping = False
        self.escape_timer = 0.0
        self.escape_angular_dir = 1.0  # +1 = left, -1 = right

        # Exploration Memory & Pacing Trap Detection
        self.visited_grid = {}        # (cell_x, cell_y) -> visit count
        self.pos_history = []         # [(time, x, y), ...]
        self.forced_bypass_dir = 0.0  # +1.0 for Left/Up, -1.0 for Right/Down
        self.forced_bypass_timer = 0.0

        # 10Hz Control Loop (0.1s interval)
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Artificial Potential Field Navigator Node initialized.")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def target_callback(self, msg: Point):
        # Only log if target actually changes by more than 5cm
        if not self.has_target or math.hypot(msg.x - self.target_x, msg.y - self.target_y) > 0.05:
            self.target_x = msg.x
            self.target_y = msg.y
            self.has_target = True
            self.get_logger().info(
                f"New target -> ({self.target_x:.2f}, {self.target_y:.2f})"
            )

    def odom_callback(self, msg: Odometry):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        # Auto-detect simulation restart / robot teleport (>0.8m sudden jump)
        if self.has_odom:
            if math.hypot(new_x - self.robot_x, new_y - self.robot_y) > 0.80:
                self.get_logger().info("Simulation restart / robot teleport detected! Resetting navigation memory.")
                self.visited_grid.clear()
                self.pos_history.clear()
                self.forced_bypass_timer = 0.0
                self.stalled_time = 0.0
                self.is_escaping = False

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
        self.front_dist = msg.x
        self.left_dist = msg.y
        self.right_dist = msg.z

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    # -----------------------------
    # Main APF Control Loop (10Hz)
    # -----------------------------
    def control_loop(self):
        if not self.has_target or not self.has_odom:
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

        # Track pacing trap (detect oscillating back-and-forth in a small region)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.pos_history.append((now_sec, self.robot_x, self.robot_y))
        self.pos_history = [(t, x, y) for (t, x, y) in self.pos_history if now_sec - t <= 2.5]

        # Decrement forced bypass timer if active
        if self.forced_bypass_timer > 0.0:
            self.forced_bypass_timer -= self.dt

        if len(self.pos_history) >= 15 and dist_to_goal > 0.8:
            old_t, old_x, old_y = self.pos_history[0]
            net_disp = math.hypot(self.robot_x - old_x, self.robot_y - old_y)
            # If net displacement is tiny (< 0.35m over 2.5s) while robot is active:
            if net_disp < 0.35 and self.forced_bypass_timer <= 0.0:
                visits_above = sum(cnt for (cx, cy), cnt in self.visited_grid.items() if cy > self.robot_y + 0.1 and abs(cx - self.robot_x) < 0.35)
                visits_below = sum(cnt for (cx, cy), cnt in self.visited_grid.items() if cy < self.robot_y - 0.1 and abs(cx - self.robot_x) < 0.35)
                # Invert direction to escape pacing trap toward UNEXPLORED space!
                self.forced_bypass_dir = -1.0 if visits_above >= visits_below else 1.0
                self.forced_bypass_timer = 4.0  # Lock escape bypass for 4.0 seconds
                self.pos_history.clear()
                self.get_logger().warn(
                    f"Pacing trap detected! Visits Above={visits_above}, Below={visits_below}. "
                    f"Breaking out: Forcing bypass {'DOWN (-Y)' if self.forced_bypass_dir < 0 else 'UP (+Y)'} for 4.0s!"
                )

        # -----------------------------
        # 1. Goal Arrival Check
        # -----------------------------
        if dist_to_goal <= goal_tol:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.vel_pub.publish(cmd)
            self.get_logger().info(
                f"Target reached! (Dist: {dist_to_goal:.2f}m <= {goal_tol:.2f}m)",
                throttle_duration_sec=2.0
            )
            self.stalled_time = 0.0
            return

        # -----------------------------
        # 2. Local Minima Escape Maneuver
        # -----------------------------
        if self.is_escaping:
            self.escape_timer -= self.dt
            if self.escape_timer <= 0.0:
                self.is_escaping = False
                self.stalled_time = 0.0
                self.get_logger().info("Escape turn complete. Re-enabling Artificial Potential Field.")
            else:
                # Force 90-degree escape rotation away from dead-end
                cmd.linear.x = 0.0
                cmd.angular.z = self.escape_angular_dir * max_ang * 0.8
                self.vel_pub.publish(cmd)
                return

        # -----------------------------
        # 3. Calculate Attraction Vector (The Pull)
        # Formula: F_att = K_att * (Target_xy - Robot_xy)
        # -----------------------------
        k_att = self.get_parameter('k_att').value
        # Cap attraction force magnitude so a distant target does not overpower obstacles
        att_mag = min(k_att * dist_to_goal, 1.20)
        f_att_world_x = att_mag * (dx_world / max(dist_to_goal, 0.001))
        f_att_world_y = att_mag * (dy_world / max(dist_to_goal, 0.001))

        # Transform Attraction Vector into the Robot's Local Frame
        # where +X is forward and +Y is left
        cos_t = math.cos(self.robot_theta)
        sin_t = math.sin(self.robot_theta)
        f_att_robot_x = f_att_world_x * cos_t + f_att_world_y * sin_t
        f_att_robot_y = -f_att_world_x * sin_t + f_att_world_y * cos_t

        # -----------------------------
        # 4. Calculate Repulsion Vectors (The Push)
        # -----------------------------
        k_rep = self.get_parameter('k_rep').value
        danger_zone = self.get_parameter('danger_zone').value
        max_rep = self.get_parameter('max_repulsion_force').value

        def calc_repulsion(d: float) -> float:
            if d >= danger_zone:
                return 0.0
            if d <= 0.04:
                return max_rep
            eff_d = max(d, 0.04)
            # Steep canal embankment: rises sharply near the banks
            ratio = (danger_zone - eff_d) / eff_d
            force = k_rep * (ratio ** 1.8)
            return min(force, max_rep)

        f_rep_front = calc_repulsion(self.front_dist)
        f_rep_left = calc_repulsion(self.left_dist)
        f_rep_right = calc_repulsion(self.right_dist)

        # Canal Streamline Bypass: when approaching a head-on bank, steer towards open / target space
        f_bypass_robot_x = 0.0
        f_bypass_robot_y = 0.0
        if f_rep_front > 0.05:
            # 1. Active forced bypass from pacing trap breakout
            if self.forced_bypass_timer > 0.0:
                target_world_y = self.forced_bypass_dir
            # 2. Hard arena boundaries: never steer into ceiling or floor
            elif self.robot_y >= 2.45:
                target_world_y = -1.0  # Near top ceiling (Y=3.0); route DOWN
            elif self.robot_y <= 0.55:
                target_world_y = 1.0   # Near bottom floor (Y=0.0); route UP
            else:
                # 3. Local Corridor Exploration (scoped strictly to current corridor, dx < 0.35m)
                visits_above = sum(cnt for (cx, cy), cnt in self.visited_grid.items() if cy > self.robot_y + 0.1 and abs(cx - self.robot_x) < 0.35)
                visits_below = sum(cnt for (cx, cy), cnt in self.visited_grid.items() if cy < self.robot_y - 0.1 and abs(cx - self.robot_x) < 0.35)

                if visits_above > visits_below + 6:
                    target_world_y = -1.0  # Upper part of this corridor explored dead-end; go DOWN
                elif visits_below > visits_above + 6:
                    target_world_y = 1.0   # Lower part of this corridor explored dead-end; go UP
                else:
                    # 4. Natural Potential: follow target attraction in Y (UP if target above, DOWN if target below)
                    target_world_y = 1.0 if dy_world >= 0.0 else -1.0

            # Smooth Streamline Flow along the wall:
            # Desired world flow along the canal is (0.0, target_world_y * bypass_magnitude).
            # Transform this streamline into the robot frame:
            #   f_bypass_robot_x = F_world_y * sin(theta)  (forward flow once aligned!)
            #   f_bypass_robot_y = F_world_y * cos(theta)  (steering torque that drops to ZERO when aligned!)
            bypass_mag = max(f_rep_front * 1.8, 1.4)
            f_bypass_robot_x = (target_world_y * bypass_mag) * sin_t
            f_bypass_robot_y = (target_world_y * bypass_mag) * cos_t

        # Exploration Potential Gradient (Repel from heavily visited regions in local corridor)
        visits_above = sum(cnt for (cx, cy), cnt in self.visited_grid.items() if cy > self.robot_y + 0.1 and abs(cx - self.robot_x) < 0.35)
        visits_below = sum(cnt for (cx, cy), cnt in self.visited_grid.items() if cy < self.robot_y - 0.1 and abs(cx - self.robot_x) < 0.35)
        visit_diff = visits_above - visits_below
        f_explore_world_y = 0.0
        # Only apply exploration pressure when navigating maze, ramp down near target (< 0.6m)
        if abs(visit_diff) >= 4 and dist_to_goal > 0.60:
            f_explore_world_y = -0.40 * math.tanh(visit_diff / 8.0)

        f_explore_robot_y = f_explore_world_y * cos_t

        # -----------------------------
        # 5. Total Resultant Vector
        # -----------------------------
        f_total_x = f_att_robot_x - f_rep_front + f_bypass_robot_x
        f_total_y = f_att_robot_y - f_rep_left + f_rep_right + f_bypass_robot_y + f_explore_robot_y

        # Forward canal flow rule: maintain downstream momentum when front path is clear
        # (front > 0.28m and not near target > 0.50m).
        # Side walls should NOT suppress forward flow — that's what steering is for.
        if self.front_dist > 0.28 and dist_to_goal > 0.50:
            f_total_x = max(f_total_x, 0.15)

        resultant_magnitude = math.hypot(f_total_x, f_total_y)
        heading_error = math.atan2(f_total_y, f_total_x)  # Heading error in robot frame

        # -----------------------------
        # 6. Convert to Motor Commands (geometry_msgs/Twist)
        # -----------------------------
        kp_ang = self.get_parameter('kp_angular').value
        alignment = max(0.0, math.cos(heading_error))

        # Absolute Impenetrable Wall Collision Prevention:
        # Only brake forward motion in these specific dangerous cases:
        # 1. Obstacle directly in front (< 0.24m)
        # 2. Actively steering LEFT into a close left wall (heading_error > +0.35 = turning left)
        # 3. Actively steering RIGHT into a close right wall (heading_error < -0.35 = turning right)
        # 4. Front obstacle critically close (< 0.12m)
        # 5. Heading error is extreme (> 1.40 rad = nearly backwards)
        # NOTE: Side walls should NEVER block forward motion when heading straight (heading_error ≈ 0).
        #       The robot MUST be able to pass through narrow slits with close side walls.
        turning_into_left = (self.left_dist <= 0.16 and heading_error > 0.35)
        turning_into_right = (self.right_dist <= 0.16 and heading_error < -0.35)
        front_critical = (self.front_dist <= 0.12)

        if self.front_dist <= 0.24 or turning_into_left or turning_into_right or front_critical or abs(heading_error) > 1.40:
            cmd.linear.x = 0.0
            cmd.angular.z = max(min(kp_ang * heading_error, max_ang), -max_ang)
        else:
            # Smooth downhill slide towards target
            front_penalty = min(1.0, max(0.2, (self.front_dist - 0.22) / 0.30))
            # Side clearance speed scaling: slow down in narrow corridors but never stop
            side_clearance = min(self.left_dist, self.right_dist)
            side_penalty = min(1.0, max(0.35, (side_clearance - 0.12) / 0.30))
            # Smooth arrival deceleration: slows robot down when within 0.60m so it docks cleanly without overshooting
            goal_decel = min(1.0, max(0.25, dist_to_goal / 0.60))
            cmd.linear.x = max_lin * (alignment ** 1.2) * front_penalty * side_penalty * goal_decel
            cmd.angular.z = max(min(kp_ang * heading_error, max_ang), -max_ang)

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
                    f"Local minimum detected! Stalled for {self.stalled_time:.1f}s. Initiating Boredom Escape Turn!"
                )
                self.is_escaping = True
                self.escape_timer = self.get_parameter('escape_duration').value
                self.escape_angular_dir = 1.0 if self.left_dist >= self.right_dist else -1.0
                return
        else:
            self.stalled_time = 0.0

        self.get_logger().info(
            f"APF: GoalDist={dist_to_goal:.2f}m | FrontClearance={self.front_dist:.2f}m -> CmdVel: v={cmd.linear.x:.2f} m/s, w={cmd.angular.z:.2f} rad/s",
            throttle_duration_sec=1.0
        )
        self.vel_pub.publish(cmd)

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