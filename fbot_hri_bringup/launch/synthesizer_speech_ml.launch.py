from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('machine', default_value='localhost'),
        DeclareLaunchArgument('speech_synthesizer_machine', default_value=LaunchConfiguration('machine')),
        DeclareLaunchArgument('audio_player_machine', default_value=LaunchConfiguration('machine')),
        DeclareLaunchArgument('use_machine', default_value='true'),

        Node(name='speech_synthesizer', 
             package='fbot_speech', 
             executable='speech_synthesizer_ml.py',
             ros_arguments = [{'machine': LaunchConfiguration('speech_synthesizer_machine')}],
             parameters=[PathJoinSubstitution(FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml'),
                         PathJoinSubstitution(FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_speech_synthesizer_ml.yaml'), 
                         ]
            ),
        Node(name='audio_player', 
             package='fbot_speech', 
             executable='audio_player.py',
             ros_arguments = [{'machine': LaunchConfiguration('audio_player_machine')}],
             parameters=[PathJoinSubstitution(FindPackageShare('fbot_hri_bringup'), 'config', 'ros.yaml'), 
                         PathJoinSubstitution(FindPackageShare('fbot_hri_bringup'), 'config', 'fbot_audio_player.yaml'), 
                        ]
            ),
    ])