from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='homework_1',
            executable='counter_node',
            name='counter_node',
            parameters=[os.path.join(
                os.path.dirname(__file__), 'counter.yaml')]
        )
    ])
