from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('challenge_config', default_value=PathJoinSubstitution(FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_stt_quiz.yaml')),
        Node(name='speech_recognizer', 
             package='fbot_speech', 
             executable='speech_recognizer.py',
             parameters=[PathJoinSubstitution(FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml'), 
                         {'challenge_config': LaunchConfiguration('challenge_config')}]
            ),
    ])