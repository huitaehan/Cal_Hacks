from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Robot 1 Arguments
        DeclareLaunchArgument(
            'robot1_source',
            default_value='http://192.168.1.101:5000/video',
            description='Camera source for Robot 1 (URL or index)'
        ),
        # Robot 2 Arguments
        DeclareLaunchArgument(
            'robot2_source',
            default_value='http://192.168.1.102:5000/video',
            description='Camera source for Robot 2 (URL or index)'
        ),
        DeclareLaunchArgument(
            'show_display',
            default_value='False',
            description='Display GUI debug windows'
        ),

        # Robot 1 Vision Node
        Node(
            package='robot_vision',
            executable='vision_node',
            name='vision_node',
            namespace='robot1',
            output='screen',
            parameters=[{
                'robot_id': 'robot1',
                'camera_source': LaunchConfiguration('robot1_source'),
                'show_debug_window': LaunchConfiguration('show_display'),
                'image_width': 640,
                'image_height': 480,
                'processing_fps': 15.0,
                'enable_victim_detection': True,
                'publish_target_location': True,
            }]
        ),

        # Robot 2 Vision Node
        Node(
            package='robot_vision',
            executable='vision_node',
            name='vision_node',
            namespace='robot2',
            output='screen',
            parameters=[{
                'robot_id': 'robot2',
                'camera_source': LaunchConfiguration('robot2_source'),
                'show_debug_window': LaunchConfiguration('show_display'),
                'image_width': 640,
                'image_height': 480,
                'processing_fps': 15.0,
                'enable_victim_detection': True,
                'publish_target_location': True,
            }]
        ),
    ])
