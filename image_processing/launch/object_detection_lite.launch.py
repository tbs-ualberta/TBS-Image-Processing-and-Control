from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument('prompt', default_value='person.phone', description='Prompt parameter'),
        DeclareLaunchArgument('target', default_value='person', description='Target parameter'),
        DeclareLaunchArgument('print_output', default_value=True, description='print_output parameter'),
        DeclareLaunchArgument('clear_output', default_value=True, description='clear_output parameter'),

        Node(
            package='image_processing',
            executable='image_processor',
            name='img_processor_node',
            output='screen'
        )
    ])
