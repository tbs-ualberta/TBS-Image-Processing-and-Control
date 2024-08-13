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
                {'target_confidence_threshold': 0.1},
                {'print_output': True},
                {'clear_output': True},
                {'regulate_process_rate': False},
                {'rgb_img_topic': 'zed/zed_node/rgb/image_rect_color'},
                {'depth_img_topic': 'zed/zed_node/depth/depth_registered'},
                {'cam_info_topic': 'zed/zed_node/depth/camera_info'}
            ]
        ),
        Node(
            package='image_processing',
            executable='ranger_control_demo',
            name='ranger_control_demo_node',
            output='screen'
        )
    ])
