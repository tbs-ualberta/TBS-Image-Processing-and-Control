from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments to enable or disable nodes
        DeclareLaunchArgument('run_img_display', default_value='false', description='Run image display node'),
        DeclareLaunchArgument('run_img_save', default_value='false', description='Run image save node'),
        DeclareLaunchArgument('run_mask_display', default_value='true', description='Run mask display node'),

        # Always run image processor
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
                {'regulate_process_rate': True},
                {'processing_rate': 1.0},
                {'rgb_img_topic': 'zed/zed_node/rgb/image_rect_color'},
                {'depth_img_topic': 'zed/zed_node/depth/depth_registered'},
                {'cam_info_topic': 'zed/zed_node/depth/camera_info'}
            ]
        ),

        # Conditional node definitions
        Node(
            condition=IfCondition(LaunchConfiguration('run_img_display')),
            package='image_processing',
            executable='image_display',
            name='img_display_node',
            output='screen'
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('run_img_save')),
            package='image_processing',
            executable='image_saver',
            name='img_save_node',
            output='screen'
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('run_mask_display')),
            package='image_processing',
            executable='mask_display',
            name='mask_display_node',
            output='screen'
        ),
    ])
