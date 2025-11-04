from launch import LaunchDescription
from launch_ros.actions import Node

"""
    Launch file for running the turtlebot odeint simulation.
"""

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_simulator',
            namespace='robot_sim_node',
            executable='robot_simulation',
            name='sim_node'
        ),
        Node(
            package='turtlebot_simulator',
            namespace='cmd_vel_publisher_node',
            executable='cmd_vel_pub',
            name='cmd_vel_pub_node'
        )
    ])