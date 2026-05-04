from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description(): 
    return LaunchDescription([
        Node(
            package="phone_controlled_astro",
            executable="phone_controlled_astro",
            name="phone_controlled_astro_node",
            output="screen",
            parameters=[{
            }],
        ),
    ])