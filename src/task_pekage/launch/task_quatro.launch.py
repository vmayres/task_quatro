from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_pekage',
            executable='serial_node_arduino',
            name='serial_node',
            output='screen'
        ),
        Node(
            package='task_pekage',
            executable='cinematica',
            name='cinematica',
            output='screen'
        )
    ])