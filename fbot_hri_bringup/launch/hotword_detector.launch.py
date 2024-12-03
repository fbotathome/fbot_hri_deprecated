from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(name='detector_hotword_node', 
             package='fbot_speech', 
             executable='detector_hotword_node',
             parameters=[PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_hotword_detection.yaml'])]),
    ])