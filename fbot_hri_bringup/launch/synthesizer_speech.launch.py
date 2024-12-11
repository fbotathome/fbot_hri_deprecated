from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_remote_ssh import NodeRemoteSSH, FindPackageShareRemote

def generate_launch_description():

    ros_config_file_arg = DeclareLaunchArgument(
        'ros_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml']),
        description='Path to the ros parameter file'
    )

    tts_config_file_arg = DeclareLaunchArgument(
        'tts_config',
        default_value=PathJoinSubstitution([FindPackageShareRemote(remote_install_space='/home/dudu/fbot_ws/install', package='fbot_hri_bringup'), 'config', 'fbot_speech_synthesizer.yaml']),
        description='Path to the tts parameter file'
    )

    audio_config_file_arg = DeclareLaunchArgument(
        'audio_config',
        default_value=PathJoinSubstitution([FindPackageShareRemote(remote_install_space='/home/dudu/fbot_ws/install', package='fbot_hri_bringup'), 'config', 'fbot_audio_player.yaml']),
        description='Path to the audio_player parameter file'
    )

    audio_player_node = Node(
        name='audio_player_node', 
        package='fbot_speech', 
        executable='audio_player',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('audio_config') 
                    ]
    )

    speech_synthesizer_node = NodeRemoteSSH(
        name='speech_synthesizer_node',
        user='dudu',
        machine='192.168.1.100',
        package='fbot_speech', 
        executable='speech_synthesizer',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('tts_config')
                    ],
        source_paths=[
                '/home/dudu/fbot_ws/install/setup.bash',
                ],
    )

    return LaunchDescription([
        ros_config_file_arg,
        audio_config_file_arg,
        tts_config_file_arg,
        audio_player_node,
        speech_synthesizer_node
    ])