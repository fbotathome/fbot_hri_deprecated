from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_remote_ssh import NodeRemoteSSH

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_machine', default_value='true'),
        DeclareLaunchArgument('speech_synthesizer_machine', default_value='jetson'),

        NodeRemoteSSH(
             name='speech_synthesizer_node',
             user='gdorneles',
             machine='192.168.1.103',
             package='fbot_speech', 
             executable='speech_synthesizer',
             ros_arguments = [{'machine': LaunchConfiguration('speech_synthesizer_machine')}],
             parameters=[PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml']),
                         PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_speech_synthesizer.yaml']), 
                         ]
            ),
        Node(name='audio_player_node', 
             package='fbot_speech', 
             executable='audio_player',
             parameters=[PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml']), 
                         PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_audio_player.yaml']), 
                        ]
            ),
    ])