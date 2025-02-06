import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='fbot_head',
            executable='emotions_bridge',
            name='emotions_bridge'),
        Node(
            package='fbot_head',
            executable='emotions_publisher',
            name='emotions_publisher')
    ])