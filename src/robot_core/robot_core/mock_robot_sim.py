import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from nav_msgs.msg import Odometry
import math
import sys

class MockRobotSim(Node):
    """
    Virtual Robot & Maze Simulator for testing APF without physical hardware.
    
    Features:
      - Simulates 2D differential drive kinematics from /cmd_vel.
      - Publishes realistic /odom at 20Hz.
      - Simulates virtual obstacles (walls) and computes raycast distances for Front, Left, Right sensors.
      - Publishes /obstacle_status (Vector3).
      - Publishes /target_location (Point).
      - Renders a live ASCII visualization directly in your terminal!
    """
    def __init__(self):
        super().__init__('mock_robot_sim')

        # -----------------------------
        # Configurable Parameters
        # -----------------------------
        self.declare_parameter('target_x', 3.5)
        self.declare_parameter('target_y', 2.0)
        self.declare_parameter('scenario', 'wall')  # 'wall', 'u_trap', or 'empty'

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.scenario = self.get_parameter('scenario').value

        # Simulated Robot Pose
        self.x = 0.5
        self.y = 1.0
        self.theta = 0.0  # Heading in radians

        # Current velocity command
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Define obstacle layout based on scenario
        self.obstacles = []
        self.setup_scenario()

        # -----------------------------
        # ROS 2 Interfaces
        # -----------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.obstacle_pub = self.create_publisher(Vector3, '/obstacle_status', 10)
        self.target_pub = self.create_publisher(Point, '/target_location', 10)

        # 20Hz physics update loop (dt = 0.05s)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_physics)

        # 2Hz terminal map renderer loop
        self.render_counter = 0

        self.get_logger().info(f"MockRobotSim started in '{self.scenario}' scenario. Target: ({self.target_x}, {self.target_y})")

    def setup_scenario(self):
        """Define line segments for obstacles [(x1, y1, x2, y2), ...]"""
        # Outer arena perimeter walls (4.8m x 3.0m area)
        arena_boundaries = [
            (0.0, 0.0, 4.8, 0.0),  # Bottom boundary
            (0.0, 3.0, 4.8, 3.0),  # Top boundary
            (0.0, 0.0, 0.0, 3.0),  # Left boundary
            (4.8, 0.0, 4.8, 3.0),  # Right boundary
        ]

        if self.scenario == 'wall':
            # Single vertical wall
            self.obstacles = [
                (2.0, 0.5, 2.0, 2.2)
            ] + arena_boundaries
        elif self.scenario in ['multilayer', 'maze', 'slalom']:
            # Double-layer alternating maze (S-Curve Slalom)
            # Wall 1 connects solidly to bottom (Y=0.0 to 2.1): opening at top (Y > 2.1)
            # Wall 2 connects solidly to top boundary (Y=0.9 to 3.0): opening at bottom (Y < 0.9)
            self.obstacles = [
                (1.7, 0.0, 1.7, 2.1),  # Layer 1: Solidly touches bottom boundary (Y=0.0)
                (2.9, 0.9, 2.9, 3.0),  # Layer 2: Solidly touches top boundary (Y=3.0)
            ] + arena_boundaries
            self.target_x = 4.0
            self.target_y = 2.0
            self.x = 0.5
            self.y = 1.0
            self.theta = 0.0
        elif self.scenario == 'corridor':
            # Doorway scenario
            self.obstacles = [
                (2.0, 0.0, 2.0, 1.1),  # Bottom wall
                (2.0, 1.8, 2.0, 3.0),  # Top wall (Doorway gap in center: 1.1m to 1.8m)
            ] + arena_boundaries
            self.target_x = 3.6
            self.target_y = 1.5
            self.x = 0.5
            self.y = 0.7
            self.theta = 0.0
        elif self.scenario == 'u_trap':
            # A U-shaped pocket to test the Boredom Timer local minima escape!
            self.obstacles = [
                (1.8, 0.8, 2.4, 0.8),  # Bottom wall of U
                (2.4, 0.8, 2.4, 2.2),  # Back wall of U (blocking target)
                (1.8, 2.2, 2.4, 2.2),  # Top wall of U
            ] + arena_boundaries
            self.target_x = 3.5
            self.target_y = 1.5
            self.x = 0.5
            self.y = 1.5
            self.theta = 0.0
        elif self.scenario in ['triple', 'gauntlet']:
            # Triple-layer slalom maze (3 alternating walls: Up -> Down -> Up)
            # Wall 1 blocks bottom (Y=0.0 to 2.1): must curve UP
            # Wall 2 blocks top (Y=0.9 to 3.0): must curve DOWN
            # Wall 3 blocks bottom (Y=0.0 to 2.1): must curve UP again
            self.obstacles = [
                (1.4, 0.0, 1.4, 2.1),  # Wall 1 (Opening at top)
                (2.5, 0.9, 2.5, 3.0),  # Wall 2 (Opening at bottom)
                (3.6, 0.0, 3.6, 2.1),  # Wall 3 (Opening at top)
            ] + arena_boundaries
            self.target_x = 4.3
            self.target_y = 1.5
            self.x = 0.4
            self.y = 1.0
            self.theta = 0.0
        elif self.scenario in ['office', 'rooms', 'building']:
            # Realistic Search-and-Rescue multi-room floorplan with hallway and doorways
            # Horizontal corridor wall (Y=1.3) with hallway entrance (X=1.8 to 2.6)
            # Vertical wall (X=2.6) dividing Room 1 and Room 2, with interior doorway (Y=2.0 to 2.5)
            self.obstacles = [
                (0.6, 1.3, 1.8, 1.3),  # Hallway wall left
                (2.6, 1.3, 4.4, 1.3),  # Hallway wall right (Doorway gap in hallway: X=1.8 to 2.6)
                (2.6, 1.3, 2.6, 1.9),  # Room divider lower segment
                (2.6, 2.5, 2.6, 3.0),  # Room divider upper segment (Interior doorway: Y=1.9 to 2.5)
            ] + arena_boundaries
            self.target_x = 3.8
            self.target_y = 2.2
            self.x = 0.5
            self.y = 0.6
            self.theta = 0.0
        elif self.scenario in ['clutter', 'rubble', 'pillars']:
            # Debris / Pillar field testing continuous obstacle flow
            self.obstacles = [
                (1.4, 0.7, 1.4, 1.3),  # Pillar 1
                (1.4, 1.8, 1.4, 2.4),  # Pillar 2
                (2.4, 1.2, 2.4, 1.8),  # Pillar 3 (center block)
                (3.3, 0.6, 3.3, 1.2),  # Pillar 4
                (3.3, 1.9, 3.3, 2.5),  # Pillar 5
            ] + arena_boundaries
            self.target_x = 4.2
            self.target_y = 1.5
            self.x = 0.5
            self.y = 1.5
            self.theta = 0.0
        else:
            self.obstacles = arena_boundaries[:]

    def cmd_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def update_physics(self):
        # 1. Kinematics integration (Euler method)
        self.theta += self.angular_z * self.dt
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Physical robot collision detection (10cm body collision radius)
        robot_radius = 0.10
        dx_step = self.linear_x * math.cos(self.theta) * self.dt
        dy_step = self.linear_x * math.sin(self.theta) * self.dt
        new_x = self.x + dx_step
        new_y = self.y + dy_step

        # Clamp strictly inside arena boundary
        new_x = max(robot_radius, min(4.8 - robot_radius, new_x))
        new_y = max(robot_radius, min(3.0 - robot_radius, new_y))

        # Strict Rule 1: Continuous Collision Detection (CCD)
        # Robot motion vector can NEVER cross any obstacle segment
        crosses_any_wall = any(
            self.segments_intersect(self.x, self.y, new_x, new_y, x1, y1, x2, y2)
            for (x1, y1, x2, y2) in self.obstacles
        )

        # Strict Rule 2: Body Distance
        # Robot center must remain at least robot_radius away from all obstacle segments
        min_wall_dist = min(
            (self.dist_point_to_segment(new_x, new_y, x1, y1, x2, y2) for (x1, y1, x2, y2) in self.obstacles),
            default=999.0
        )

        if not crosses_any_wall and min_wall_dist >= robot_radius:
            self.x = new_x
            self.y = new_y
        else:
            # Collision occurred: attempt sliding along obstacle tangent ONLY if safe
            closest_obs = min(
                self.obstacles,
                key=lambda obs: self.dist_point_to_segment(self.x, self.y, obs[0], obs[1], obs[2], obs[3])
            ) if self.obstacles else None

            if closest_obs is not None:
                ox1, oy1, ox2, oy2 = closest_obs
                wall_dx = ox2 - ox1
                wall_dy = oy2 - oy1
                wall_len = math.hypot(wall_dx, wall_dy)
                if wall_len > 1e-6:
                    tx = wall_dx / wall_len
                    ty = wall_dy / wall_len
                    # Project motion step onto wall tangent vector
                    slide_amount = dx_step * tx + dy_step * ty
                    slide_x = self.x + slide_amount * tx
                    slide_y = self.y + slide_amount * ty

                    # Clamp slide inside arena
                    slide_x = max(robot_radius, min(4.8 - robot_radius, slide_x))
                    slide_y = max(robot_radius, min(3.0 - robot_radius, slide_y))

                    slide_crosses = any(
                        self.segments_intersect(self.x, self.y, slide_x, slide_y, x1, y1, x2, y2)
                        for (x1, y1, x2, y2) in self.obstacles
                    )
                    curr_d = self.dist_point_to_segment(self.x, self.y, ox1, oy1, ox2, oy2)
                    slide_d = self.dist_point_to_segment(slide_x, slide_y, ox1, oy1, ox2, oy2)
                    slide_min_all = min(
                        (self.dist_point_to_segment(slide_x, slide_y, x1, y1, x2, y2) for (x1, y1, x2, y2) in self.obstacles),
                        default=999.0
                    )

                    # Only accept slide if it does not cross walls and preserves safe clearance from ALL obstacles
                    if not slide_crosses and slide_d >= min(curr_d, robot_radius * 0.98) and slide_min_all >= robot_radius * 0.95:
                        self.x = slide_x
                        self.y = slide_y
                    # Otherwise, hard block: robot stops dead (self.x, self.y unchanged)

        # 2. Publish /target_location continuously
        target_msg = Point()
        target_msg.x = self.target_x
        target_msg.y = self.target_y
        target_msg.z = 0.0
        self.target_pub.publish(target_msg)

        # 3. Publish /odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion (yaw only)
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom_msg)

        # 4. Raycast distance sensors to virtual obstacles (with realistic FOV cone)
        # Front sensor checks central ray and bumper corners (+-16 deg) so corners don't clip walls
        front_dist = min(
            self.raycast(self.theta),
            self.raycast(self.theta + 0.28),
            self.raycast(self.theta - 0.28)
        )
        left_dist = min(
            self.raycast(self.theta + math.pi / 2.0),
            self.raycast(self.theta + math.pi / 3.0)
        )
        right_dist = min(
            self.raycast(self.theta - math.pi / 2.0),
            self.raycast(self.theta - math.pi / 3.0)
        )

        obs_msg = Vector3()
        obs_msg.x = front_dist
        obs_msg.y = left_dist
        obs_msg.z = right_dist
        self.obstacle_pub.publish(obs_msg)

        # 5. Render live status in terminal
        self.render_counter += 1
        if self.render_counter >= 10:  # Every 0.5 seconds
            self.render_counter = 0
            self.render_terminal_map(front_dist, left_dist, right_dist)

    def raycast(self, ray_angle: float, max_range: float = 2.0) -> float:
        """Calculate minimum distance from robot to any obstacle along a ray."""
        min_dist = max_range
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)

        for (x1, y1, x2, y2) in self.obstacles:
            # Ray-segment intersection
            d = self.intersect_ray_segment(self.x, self.y, dx, dy, x1, y1, x2, y2)
            if d is not None and 0.0 <= d < min_dist:
                min_dist = d

        return min_dist

    @staticmethod
    def intersect_ray_segment(px, py, dx, dy, x1, y1, x2, y2):
        """Ray: P + t*D (t >= 0). Segment: A + u*(B - A) (0 <= u <= 1)."""
        sx = x2 - x1
        sy = y2 - y1
        denom = dx * sy - dy * sx
        if abs(denom) < 1e-6:
            return None  # Parallel

        t = ((x1 - px) * sy - (y1 - py) * sx) / denom
        u = ((x1 - px) * dy - (y1 - py) * dx) / denom

        if t >= 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    @staticmethod
    def dist_point_to_segment(px, py, x1, y1, x2, y2):
        """Shortest distance from point (px, py) to line segment (x1, y1) -> (x2, y2)."""
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx*dx + dy*dy
        if l2 == 0:
            return math.hypot(px - x1, py - y1)
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    @staticmethod
    def segments_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
        """Returns True if segment (x1,y1)-(x2,y2) intersects segment (x3,y3)-(x4,y4)."""
        def cross(ax, ay, bx, by, cx, cy):
            return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)

        d1 = cross(x3, y3, x4, y4, x1, y1)
        d2 = cross(x3, y3, x4, y4, x2, y2)
        d3 = cross(x1, y1, x2, y2, x3, y3)
        d4 = cross(x1, y1, x2, y2, x4, y4)

        if ((d1 > 1e-6 and d2 < -1e-6) or (d1 < -1e-6 and d2 > 1e-6)) and \
           ((d3 > 1e-6 and d4 < -1e-6) or (d3 < -1e-6 and d4 > 1e-6)):
            return True

        def on_seg(px, py, ax, ay, bx, by):
            return (min(ax, bx) - 1e-4 <= px <= max(ax, bx) + 1e-4 and
                    min(ay, by) - 1e-4 <= py <= max(ay, by) + 1e-4 and
                    abs(cross(ax, ay, bx, by, px, py)) < 1e-4)

        return (on_seg(x1, y1, x3, y3, x4, y4) or
                on_seg(x2, y2, x3, y3, x4, y4) or
                on_seg(x3, y3, x1, y1, x2, y2) or
                on_seg(x4, y4, x1, y1, x2, y2))

    def render_terminal_map(self, front: float, left: float, right: float):
        """Render a clean 2D ASCII mini-map showing Robot, Obstacles, and Target."""
        width = 34
        height = 13
        x_max = 4.8
        y_max = 3.0

        grid = [[' ' for _ in range(width)] for _ in range(height)]

        def world_to_grid(wx, wy):
            gx = int((wx / x_max) * (width - 1))
            gy = int(((y_max - wy) / y_max) * (height - 1))
            return max(0, min(width - 1, gx)), max(0, min(height - 1, gy))

        # Draw obstacles (skip perimeter boundaries so they don't overwrite inner map)
        for (x1, y1, x2, y2) in self.obstacles:
            if (y1 == y2 == 0.0) or (y1 == y2 == 3.0) or (x1 == x2 == 0.0) or (x1 == x2 == 4.8):
                continue
            steps = 25
            for s in range(steps + 1):
                interp_x = x1 + (x2 - x1) * (s / steps)
                interp_y = y1 + (y2 - y1) * (s / steps)
                gx, gy = world_to_grid(interp_x, interp_y)
                grid[gy][gx] = '#'

        # Draw Target
        tx, ty = world_to_grid(self.target_x, self.target_y)
        grid[ty][tx] = 'T'

        # Draw Robot
        rx, ry = world_to_grid(self.x, self.y)
        heading_deg = int(math.degrees(self.theta)) % 360
        # Robot directional icon
        if 45 <= heading_deg < 135:
            r_char = '^'
        elif 135 <= heading_deg < 225:
            r_char = '<'
        elif 225 <= heading_deg < 315:
            r_char = 'v'
        else:
            r_char = '>'
        grid[ry][rx] = r_char

        # Distance to goal
        dist_to_goal = math.hypot(self.target_x - self.x, self.target_y - self.y)

        # Print output
        print("\033[H\033[J", end="")  # Clear terminal
        print("=" * 40)
        print(f" 🤖 MOCK ROBOT APF SIMULATOR ({self.scenario.upper()})")
        print("=" * 40)
        border = "+" + "-" * width + "+"
        print(border)
        for row in grid:
            print("|" + "".join(row) + "|")
        print(border)
        print(f" Robot Pose:  X: {self.x:5.2f}m, Y: {self.y:5.2f}m | Heading: {heading_deg:3d}°")
        print(f" Goal Dist:   {dist_to_goal:5.2f}m  | Target: ({self.target_x:.1f}, {self.target_y:.1f})")
        print(f" Clearances:  Front: {front:4.2f}m | Left: {left:4.2f}m | Right: {right:4.2f}m")
        print(f" Motors Cmd:  linear.x = {self.linear_x:5.2f} m/s | angular.z = {self.angular_z:5.2f} rad/s")
        print("=" * 40)
        if dist_to_goal <= 0.15:
            print(" 🎯 TARGET REACHED! (Robot stopped successfully)")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = MockRobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
