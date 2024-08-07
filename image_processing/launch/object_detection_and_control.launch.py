from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_processing',
            executable='image_processor',
            name='img_processor_node',
            output='screen',
            parameters=[
                {'prompt': 'person.phone'},
                {'target': 'person'},
                {'print_output': 'true'},
                {'clear_output': 'true'},
                {'rgb_img_topic': 'zed/zed_node/rgb/image_rect_color'},
                {'depth_img_topic': 'zed/zed_node/depth/depth_registered'}
            ]
        ),
        Node(
            package='image_processing',
            executable='ranger_control_demo',
            name='ranger_control_demo_node',
            output='screen'
        )
    ])
