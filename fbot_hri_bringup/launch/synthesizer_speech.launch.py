from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_remote_ssh import NodeRemoteSSH, FindPackageShareRemote

def generate_launch_description():
    return LaunchDescription([
        NodeRemoteSSH(
             name='speech_synthesizer_node',
             user='dudu',
             machine='192.168.1.100',
             package='fbot_speech', 
             executable='speech_synthesizer',
             parameters=[PathJoinSubstitution([FindPackageShareRemote(remote_install_space='/home/dudu/fbot_ws/install', package='fbot_hri_bringup'), 'config', 'ros.yaml']),
                         PathJoinSubstitution([FindPackageShareRemote(remote_install_space='/home/dudu/fbot_ws/install', package='fbot_hri_bringup'), 'config', 'fbot_speech_synthesizer.yaml']), 
                         ],
             source_paths=[
                        '/home/dudu/fbot_ws/install/setup.bash',
                        ],
            ),
        Node(name='audio_player_node', 
             package='fbot_speech', 
             executable='audio_player',
             parameters=[PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml']), 
                         PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_audio_player.yaml']), 
                        ]
            ),
    ])