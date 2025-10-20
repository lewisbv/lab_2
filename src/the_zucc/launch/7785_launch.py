from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='the_zucc',
            executable='detect',
            name='detect_object',
            output='screen'
        ),
        Node(
            package='the_zucc',
            executable='rotate',
            name='rotate_robot',
            output='screen'
        )
    ])
