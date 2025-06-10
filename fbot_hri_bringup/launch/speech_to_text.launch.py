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

    ros_config_file_remote_arg = DeclareLaunchArgument(
        'ros_config_remote',
        default_value=ros_config_path_remote,
        description='Path to the ros parameter file'
    )
    challenge_config_path = PathJoinSubstitution([
        get_package_share_directory('fbot_hri_bringup'),
        'config',
        LaunchConfiguration('challenge')]
    )

    challenge_config_path_remote = PathJoinSubstitution([
        FindPackageShareRemote(remote_install_space='/home/jetson/jetson_ws/install', package='fbot_hri_bringup'),
        'config',
        LaunchConfiguration('challenge')]
    )

    challenge_config = DeclareLaunchArgument(
        'challenge_config', 
        default_value=challenge_config_path,
        description='Path to the challenge parameter file'
    )

    challenge_config_remote = DeclareLaunchArgument(
        'challenge_config_remote', 
        default_value=challenge_config_path_remote,
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

    stt_remote_node = NodeRemoteSSH(
        name='speech_recognizer_node',
        package='fbot_speech',
        executable='speech_recognizer',
        parameters=[LaunchConfiguration('ros_config_remote'),
                    LaunchConfiguration('challenge_config_remote')
                    ],
        user='jetson',
        machine="jetson",
        source_paths=[
            "/home/jetson/jetson_ws/install/setup.bash"
        ],
        condition=IfCondition(LaunchConfiguration('use_remote'))
    )

    return LaunchDescription([
        ros_file_arg,
        challenge_config,
        stt_node
    ])