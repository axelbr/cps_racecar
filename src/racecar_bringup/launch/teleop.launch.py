from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[LaunchConfiguration('teleop_config')]
    )

    joy_teleop_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[LaunchConfiguration('teleop_config')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('teleop_config', default_value='', description='Teleop configuration file.'),
        joy_node,
        joy_teleop_node
    ])