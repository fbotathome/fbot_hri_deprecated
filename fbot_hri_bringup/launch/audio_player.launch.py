from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ros_config_file_arg = DeclareLaunchArgument(
        'ros_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml']),
        description='Path to the ros parameter file'
    )

    audio_config_file_arg = DeclareLaunchArgument(
        'audio_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_audio_player.yaml']),
        description='Path to the audio_player parameter file'
    )


    audio_player_node = Node(
        name='audio_player_node', 
        package='fbot_speech', 
        executable='audio_player',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('audio_config'), 
                    ]
    )

    return LaunchDescription([
        ros_config_file_arg,
        audio_config_file_arg,
        audio_player_node,
    ])