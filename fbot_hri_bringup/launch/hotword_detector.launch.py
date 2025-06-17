from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    hotword_config_arg = DeclareLaunchArgument(
        'hotword_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', LaunchConfiguration('hotword_config_file')]),
        description='Path to the ros parameter file'
    )

    hotword_node = Node(
        name='detector_hotword_node', 
        package='fbot_speech', 
        executable='detector_hotword_node',
        parameters=[LaunchConfiguration('hotword_config'), 
                    ]
    )

    return LaunchDescription([
        hotword_config_arg,
        hotword_node
    ])