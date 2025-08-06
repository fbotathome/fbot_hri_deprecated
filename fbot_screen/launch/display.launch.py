from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    #Generates the launch description for the media display package.   
    ros_node_port = 9090  # Default rosbridge port 

    return LaunchDescription([
        # Start rosbridge_server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server',
            output='screen',
            parameters=[{'port': ros_node_port}]
        ),

        # Start the display_node 
        Node(
            package='media_display',
            executable='display_node',
            name='media_display_node',
            output='screen'
        ),

        # Automatically open Foxglove Studio in the browser
        ExecuteProcess(
            cmd=[
                'xdg-open',
                f'https://studio.foxglove.dev/?ds=rosbridge-websocket&ds.url=ws://localhost:{ros_node_port}'
            ],
            output='screen'
        )
    ])