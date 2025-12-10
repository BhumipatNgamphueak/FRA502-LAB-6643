from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim_plus',
            namespace='',
            executable='turtlesim_plus_node.py',
            name='turtlesim'
        ),
        Node(
            package='lab2',
            namespace='',
            executable='eater.py',
            name='eater'
        ),
          Node(
            package='lab2',
            namespace='',
            executable='killer.py',
            name='killer'
        ),
    ])