from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(name='speech_recognizer_node', 
             package='fbot_speech', 
             executable='speech_recognizer_old',
             parameters=[PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml']), 
                         PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_speech_recognizer_old.yaml'])]
            ),
    ])