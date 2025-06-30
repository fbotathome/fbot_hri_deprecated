from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ros_file_arg = DeclareLaunchArgument(
        'ros_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml']),
        description='Path to the ros parameter file'
    )

    challenge_config = DeclareLaunchArgument(
        'challenge_config', 
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', LaunchConfiguration('stt_config_file')]),
        description='Path to the challenge parameter file'
    )

    stt_node = Node(
        name='speech_recognizer_node', 
        package='fbot_speech', 
        executable='speech_recognizer',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('challenge_config') 
                    ]
    )


    return LaunchDescription([
        ros_file_arg,
        challenge_config,
        stt_node
    ])