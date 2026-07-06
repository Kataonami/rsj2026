from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='joy_node_interface',
            executable='joy_node_interface',
            name='joy_node_interface',
            output='screen',
        ),
    ])
