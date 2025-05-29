from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_remote_ssh import NodeRemoteSSH, FindPackageShareRemote

def generate_launch_description():
    
    hot_config_path = PathJoinSubstitution([
        get_package_share_directory('fbot_hri_bringup'),
        'config',
        'ros.yaml']
    )

    hot_config_path_remote = PathJoinSubstitution([
        FindPackageShareRemote(remote_install_space='/home/jetson/jetson_ws/install', package='fbot_hri_bringup'),
        'config',
        'ros.yaml']
    )

    hot_config_file_remote_arg = DeclareLaunchArgument(
        'hot_config_remote',
        default_value=hot_config_path_remote,
        description='Path to the ros parameter file'
    )

    hot_config_file_arg = DeclareLaunchArgument(
        'hot_config',
        default_value=hot_config_path,
        description='Path to the ros parameter file'
    )

    hotword_remote_node = NodeRemoteSSH(
        name='detector_hotword_node', 
        package='fbot_speech', 
        executable='detector_hotword_node',
        parameters=[LaunchConfiguration('hot_config_remote')
                    ],
        user='jetson',
        machine="jetson",
        source_paths=[
            "/home/jetson/jetson_ws/install/setup.bash"
        ],
        condition=IfCondition(LaunchConfiguration('use_remote'))
    )

    hotword_node = Node(
        name='detector_hotword_node', 
        package='fbot_speech', 
        executable='detector_hotword_node',
        parameters=[LaunchConfiguration('hot_config'),
                    ],
        condition=UnlessCondition(LaunchConfiguration('use_remote'))
    )

    return LaunchDescription([
        hot_config_file_arg,
        hot_config_file_remote_arg,
        hotword_remote_node,
        hotword_node
    ])