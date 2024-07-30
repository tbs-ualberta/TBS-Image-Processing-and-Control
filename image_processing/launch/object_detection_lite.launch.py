from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument('prompt', default_value='person.phone', description='Prompt parameter'),
        DeclareLaunchArgument('target', default_value='person', description='Target parameter'),

        Node(
            package='image_processing',
            executable='img_processor_node.py',
            name='img_processor_node',
            output='screen'
        )
    ])
