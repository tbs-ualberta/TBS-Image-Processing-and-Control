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
            executable='image_processor',
            name='img_processor_node',
            output='screen'
        )
    ])
