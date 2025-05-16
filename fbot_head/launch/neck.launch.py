import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        
    return launch.LaunchDescription([
        Node(
            package='fbot_head',
            executable='neck_controller',
            name='neck_controller')
    ])