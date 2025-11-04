from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""
Launch file for running the turtlebot ode simulation (single binary running 2 nodes).
"""

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value='/home/lorenzo/polimi/mobile_robots/Control-Of-Mobile-Robots/ros_ws/src/turtlebot_simulator/config/params.yaml', # Change this to your actual path
            description='Path to the ROS2 parameters YAML file'
        ),
        Node(
            package='turtlebot_simulator',
            executable='robot_sim_node',
            parameters=[LaunchConfiguration('params_file')]
        ),
        Node(
            package='turtlebot_simulator',
            executable='cmd_vel_pub_node',
        )
    ])