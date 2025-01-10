import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='capacitacion_pkg',          # Your package name
            executable='Com_motores.py',         # Python file to execute
            name='com_motores_node',             # Node name
            output='screen',                     # Output to the screen
        ),
        Node(
            package='capacitacion_pkg',          # Your package name
            executable='Publisher_msg.py',       # Python file to execute
            name='publisher_msg_node',           # Node name
            output='screen',                     # Output to the screen
        ),
    ])
