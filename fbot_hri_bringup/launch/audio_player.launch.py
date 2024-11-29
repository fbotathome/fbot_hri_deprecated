from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(name='audio_player', package='fbot_speech', executable='audio_player.py'),
    ])