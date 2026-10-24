from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='robot1',
            description='Namespace ID for the robot'
        ),
        DeclareLaunchArgument(
            'camera_source',
            default_value='0',
            description='Webcam index (0) or stream URL (e.g. http://192.168.1.101:5000/video)'
        ),
        DeclareLaunchArgument(
            'show_display',
            default_value='False',
            description='Display GUI debug window on GX10 desktop'
        ),
        Node(
            package='robot_vision',
            executable='vision_node',
            name='vision_node',
            namespace=LaunchConfiguration('robot_id'),
            output='screen',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'camera_source': LaunchConfiguration('camera_source'),
                'show_debug_window': LaunchConfiguration('show_display'),
                'image_width': 640,
                'image_height': 480,
                'processing_fps': 20.0,
                'enable_victim_detection': True,
                'publish_target_location': True,
            }]
        )
    ])
