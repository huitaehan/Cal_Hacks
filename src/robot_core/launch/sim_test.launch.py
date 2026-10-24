from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_core',
            executable='navigator_node',
            name='navigator_node',
            output='screen',
            parameters=[{
                'k_att': 0.5,
                'k_rep': 0.08,
                'danger_zone': 0.50,
                'max_linear_vel': 0.30,
                'max_angular_vel': 1.20,
            }]
        ),
        Node(
            package='robot_core',
            executable='mock_robot_sim',
            name='mock_robot_sim',
            output='screen',
            parameters=[{
                'scenario': 'wall',  # Options: 'wall', 'u_trap', 'empty'
                'target_x': 3.5,
                'target_y': 2.0,
            }]
        ),
    ])
