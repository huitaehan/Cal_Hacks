from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_raw',
            description='Topic published by Raspberry Pi camera node'
        ),
        DeclareLaunchArgument(
            'publish_debug',
            default_value='True',
            description='Publish /vision/wall_debug visual image'
        ),
        Node(
            package='robot_vision',
            executable='visual_wall_detector',
            name='visual_wall_detector',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'obstacle_topic': '/obstacle_status',
                'debug_image_topic': '/vision/wall_debug',
                'publish_debug_image': LaunchConfiguration('publish_debug'),
                'canny_low': 40,
                'canny_high': 120,
                'roi_floor_ratio': 0.50,
                'max_detection_range': 2.50,
                'min_detection_range': 0.15,
            }]
        )
    ])
