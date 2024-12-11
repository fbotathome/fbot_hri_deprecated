from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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

    tts_config_file_arg = DeclareLaunchArgument(
        'tts_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_speech_synthesizer_old.yaml']),
        description='Path to the tts parameter file'
    )

    audio_player_node = Node(
        name='audio_player_node', 
        package='fbot_speech', 
        executable='audio_player',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('audio_config') 
                    ]
    )

    speech_synthesizer_node = Node(
        name='speech_synthesizer_node', 
        package='fbot_speech', 
        executable='speech_synthesizer_old',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('tts_config') 
                    ]
    )

    return LaunchDescription([
        ros_config_file_arg,
        audio_config_file_arg,
        tts_config_file_arg,
        audio_player_node,
        speech_synthesizer_node
    ])