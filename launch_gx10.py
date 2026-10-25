#!/usr/bin/env python3
"""
Unified Single-Process Brain for ASUS GX10 Base Station.
Runs both:
  1. VisualWallDetector: Camera floor horizon edge scan + Ultrasonic fusion + Person AI -> /obstacle_status
  2. NavigatorNode: Artificial Potential Field pathfinding + Exploration -> /cmd_vel

Advantages:
  - Zero IPC / Docker container separation issues (runs in 1 process).
  - Microsecond latency between perception and pathfinding.
  - Formatted live status monitor in your terminal.
  - Clean shutdown on Ctrl+C.
"""

import sys
import os
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Add module paths so script runs cleanly from anywhere
_curr_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_curr_dir, 'src', 'robot_vision', 'robot_vision'))
sys.path.insert(0, os.path.join(_curr_dir, 'src', 'robot_core', 'robot_core'))

try:
    from visual_wall_detector import VisualWallDetector
except ImportError:
    # Direct file import fallback
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "visual_wall_detector",
        os.path.join(_curr_dir, 'src', 'robot_vision', 'robot_vision', 'visual_wall_detector.py')
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    VisualWallDetector = mod.VisualWallDetector

try:
    from navigator_node import NavigatorNode
except ImportError:
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "navigator_node",
        os.path.join(_curr_dir, 'src', 'robot_core', 'robot_core', 'navigator_node.py')
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    NavigatorNode = mod.NavigatorNode


def main(args=None):
    rclpy.init(args=args)

    print("\n" + "=" * 64)
    print("      ASUS GX10 SEARCH-AND-RESCUE AI BASE STATION")
    print("=" * 64)
    print(" [1/2] Initializing Visual Wall Detector (OpenCV + AI)...")
    vision_node = VisualWallDetector()

    print(" [2/2] Initializing APF Navigator (Pathfinding + Motor Brain)...")
    nav_node = NavigatorNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(vision_node)
    executor.add_node(nav_node)

    print("\n" + "-" * 64)
    print(" [READY] Perception and Navigation Active in Single Process!")
    print("         Subscribed: /image_raw/compressed & /ultrasonic_center/left/right")
    print("         Publishing: /obstacle_status & /cmd_vel")
    print("         Press Ctrl+C to shut down cleanly.")
    print("-" * 64 + "\n")

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n[GX10] Shutting down AI base station cleanly...")
    finally:
        vision_node.destroy_node()
        nav_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[GX10] AI Base Station stopped. Goodbye.\n")


if __name__ == '__main__':
    main()
