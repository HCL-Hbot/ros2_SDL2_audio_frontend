from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('sdl2_audio_frontend')
    audio_config = os.path.join(pkg_dir, 'config', 'audio_params.yaml')
    vad_config = os.path.join(pkg_dir, 'config', 'vad_params.yaml')
    
    return LaunchDescription([
        Node(
            package='sdl2_audio_frontend',
            executable='audio_capture_node',
            name='audio_capture_node',
            parameters=[audio_config],
            output='screen'
        ),
        Node(
            package='sdl2_audio_frontend',
            executable='vad_node',
            name='vad_node',
            parameters=[vad_config],
            output='screen'
        )
    ])