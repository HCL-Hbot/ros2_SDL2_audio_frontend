from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('sdl2_audio_frontend')
    config_file = os.path.join(pkg_dir, 'config', 'audio_params.yaml')

    return LaunchDescription([
        # Launch audio capture node with parameters
        Node(
            package='sdl2_audio_frontend',
            executable='audio_capture_node',
            name='audio_capture_node',
            parameters=[config_file],
            output='screen',
        ),
    ])