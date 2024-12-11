from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ros_file_arg = DeclareLaunchArgument(
        'ros_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml']),
        description='Path to the ros parameter file'
    )

    stt_file_arg = DeclareLaunchArgument(
        'stt_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_speech_recognizer_old.yaml']),
        description='Path to the stt parameter file'
    )

    stt_node = Node(
        name='speech_recognizer_node', 
        package='fbot_speech', 
        executable='speech_recognizer_old',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('stt_config') 
                    ]
    )

    return LaunchDescription([
        ros_file_arg,
        stt_file_arg,
        stt_node
    ])