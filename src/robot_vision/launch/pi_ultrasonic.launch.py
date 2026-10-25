from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'trig_front',
            default_value='23',
            description='BCM GPIO pin for Front Trigger'
        ),
        DeclareLaunchArgument(
            'echo_front',
            default_value='24',
            description='BCM GPIO pin for Front Echo'
        ),
        DeclareLaunchArgument(
            'enable_sides',
            default_value='False',
            description='Enable Left and Right side HC-SR04 sensors'
        ),
        Node(
            package='robot_vision',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen',
            parameters=[{
                'trig_front': LaunchConfiguration('trig_front'),
                'echo_front': LaunchConfiguration('echo_front'),
                'enable_side_sensors': LaunchConfiguration('enable_sides'),
                'publish_rate_hz': 15.0,
                'min_range_m': 0.02,
                'max_range_m': 3.00,
                'median_filter_size': 3,
            }]
        )
    ])
