from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument('prompt', default_value='person.phone', description='Prompt parameter (multiple objects can be detected using a \'.\' as a separator)'),
        DeclareLaunchArgument('target', default_value='person', description='Target parameter (must be contained in the prompt)'),
        DeclareLaunchArgument('print_output', default_value='true', description='print_output parameter'),
        DeclareLaunchArgument('clear_output', default_value='true', description='clear_output parameter'),
        DeclareLaunchArgument('rgb_img_topic', default_value='zed/zed_node/rgb/image_rect_color', description='topic for rgb image publishing'),
        DeclareLaunchArgument('depth_img_topic', default_value='zed/zed_node/depth/depth_registered', description='topic for registered depth image publishing'),

        Node(
            package='image_processing',
            executable='image_processor',
            name='img_processor_node',
            output='screen',
            parameters=[
                {'prompt': LaunchConfiguration('prompt')},
                {'target': LaunchConfiguration('target')},
                {'print_output': LaunchConfiguration('print_output')},
                {'clear_output': LaunchConfiguration('clear_output')},
                {'rgb_img_topic': LaunchConfiguration('rgb_img_topic')},
                {'depth_img_topic': LaunchConfiguration('depth_img_topic')}
            ]
        )
    ])
