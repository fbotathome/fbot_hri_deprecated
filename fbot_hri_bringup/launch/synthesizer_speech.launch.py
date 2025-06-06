from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_remote_ssh import NodeRemoteSSH, FindPackageShareRemote

def generate_launch_description():
    ros_config_path_remote = PathJoinSubstitution([
        FindPackageShareRemote(remote_install_space='/home/jetson/jetson_ws/install', package='fbot_hri_bringup'),
        'config',
        'ros.yaml']
    )

    ros_config_path = PathJoinSubstitution([
        get_package_share_directory('fbot_hri_bringup'),
        'config',
        'ros.yaml']
    )

    ros_config_file_remote_arg = DeclareLaunchArgument(
        'ros_config_remote',
        default_value=ros_config_path_remote,
        description='Path to the ros parameter file'
    )

    ros_config_file_arg = DeclareLaunchArgument(
        'ros_config',
        default_value=ros_config_path,
        description='Path to the ros parameter file'
    )

    tts_config_file_path_remote = PathJoinSubstitution([
        FindPackageShareRemote(remote_install_space='/home/jetson/jetson_ws/install', package='fbot_hri_bringup'),
        'config',
        'fbot_speech_synthesizer.yaml']
    )

    tts_config_file_path = PathJoinSubstitution([
        get_package_share_directory('fbot_hri_bringup'),
        'config',
        'fbot_speech_synthesizer.yaml']
    )

    tts_config_file_remote_arg = DeclareLaunchArgument(
        'tts_config_remote',
        default_value=tts_config_file_path_remote,
        description='Path to the tts parameter file'
    )

    tts_config_file_arg = DeclareLaunchArgument(
        'tts_config',
        default_value=tts_config_file_path,
        description='Path to the tts parameter file'
    )

    audio_config_file_path = PathJoinSubstitution([
        get_package_share_directory('fbot_hri_bringup'),
        'config',
        'fbot_audio_player.yaml'
    ])

    audio_config_file_remote_path =  PathJoinSubstitution([
        FindPackageShareRemote(remote_install_space='/home/jetson/jetson_ws/install', package='fbot_hri_bringup'),
        'config',
        'fbot_audio_player.yaml']
    )

    audio_config_file_arg = DeclareLaunchArgument(
        'audio_config',
        default_value=audio_config_file_path,
        description='Path to the audio_player parameter file'
    )

    audio_config_file_remote_arg = DeclareLaunchArgument(
        'audio_config_remote',
        default_value=audio_config_file_remote_path,
        description='Path to the audio_player parameter file'
    )

    audio_player_remote_node = NodeRemoteSSH(
        name='audio_player_node', 
        package='fbot_speech', 
        executable='audio_player',
        parameters=[LaunchConfiguration('ros_config_remote'),
                    LaunchConfiguration('audio_config_remote') 
                    ],
        user='jetson',
        machine="jetson",
        source_paths=[
            "/home/jetson/jetson_ws/install/setup.bash"
        ],
        condition=IfCondition(LaunchConfiguration('use_remote'))
    )

    audio_player_node = Node(
        name='audio_player_node', 
        package='fbot_speech', 
        executable='audio_player',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('audio_config') 
                    ],
        condition=UnlessCondition(LaunchConfiguration('use_remote'))
    )

    speech_synthesizer_remote_node = NodeRemoteSSH(
        name='speech_synthesizer_node',
        package='fbot_speech', 
        executable='speech_synthesizer',
        parameters=[LaunchConfiguration('ros_config_remote'),
                    LaunchConfiguration('tts_config_remote')
                    ],
        user='jetson',
        machine="jetson",
        source_paths=[
            "/home/jetson/jetson_ws/install/setup.bash"
        ],
        condition=IfCondition(LaunchConfiguration('use_remote'))
    )

    speech_synthesizer_node = Node(
        name='speech_synthesizer_node',
        package='fbot_speech', 
        executable='speech_synthesizer',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('tts_config')
                    ],
        condition=UnlessCondition(LaunchConfiguration('use_remote'))
    )

    return LaunchDescription([
        ros_config_file_arg,
        ros_config_file_remote_arg,
        audio_config_file_arg,audio_config_file_arg,
        audio_config_file_remote_arg,
        tts_config_file_arg,
        tts_config_file_remote_arg,
        audio_player_node,
        audio_player_remote_node,
        speech_synthesizer_remote_node,
        speech_synthesizer_node
    ])