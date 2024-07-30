from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument('prompt', default_value='person.phone', description='Prompt parameter'),
        DeclareLaunchArgument('target', default_value='person', description='Target parameter'),

        # Declare arguments to enable or disable nodes
        DeclareLaunchArgument('run_img_display', default_value='false', description='Run image display node'),
        DeclareLaunchArgument('run_img_save', default_value='false', description='Run image save node'),
        DeclareLaunchArgument('run_mask_display', default_value='true', description='Run mask display node'),

        # Always run image processor
        Node(
            package='image_processing',
            executable='img_processor_node.py',
            name='img_processor_node',
            output='screen'
        ),

        # Conditional node definitions
        Node(
            condition=LaunchConfiguration('run_img_display'),
            package='image_processing',
            executable='img_display_node.py',
            name='img_display_node',
            output='screen'
        ),
        Node(
            condition=LaunchConfiguration('run_img_save'),
            package='image_processing',
            executable='img_save_node.py',
            name='img_save_node',
            output='screen'
        ),
        Node(
            condition=LaunchConfiguration('run_mask_display'),
            package='image_processing',
            executable='mask_display_node.py',
            name='mask_display_node',
            output='screen'
        ),
    ])
