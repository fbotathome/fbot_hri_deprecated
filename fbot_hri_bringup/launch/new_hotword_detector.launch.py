from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    hot_config_file_arg = DeclareLaunchArgument(
        'hot_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_hotword_detection.yaml']),
        description='Path to the ros parameter file'
    )

    hotword_node = Node(
        name='detector_hotword_node', 
        package='fbot_speech', 
        executable='new_detector_hotword_node',
        parameters=[LaunchConfiguration('hot_config'), 
                    ]
    )

    return LaunchDescription([
        hot_config_file_arg,
        hotword_node
    ])