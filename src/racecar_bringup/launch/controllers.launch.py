from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():

    controller_config = LaunchConfiguration('controller_config')

    # Compile robot description
    robot_description = Command(['xacro ', LaunchConfiguration('model')])
   
    # Controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_config
            
        ],
        output='screen'
    )

    # Spawns state broadcaster for the hardware interface
    power_train_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["power_train_broadcaster", "--controller-manager", "/controller_manager"],
    )

    vesc_imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["vesc_imu_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawns a controller for the hardware interface
    teleop_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["teleop_controller", "-c", "/controller_manager"],
    )
    
    # Delays the launch of the controller until the broadcaster is spawned
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=power_train_broadcaster_spawner,
            on_exit=[teleop_controller_spawner],
        )
    )


    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='', description='Path to urdf file.'),
        DeclareLaunchArgument('controller_config', default_value='', description='Controller configuration file.'),
        controller_manager,
        vesc_imu_broadcaster_spawner,
        power_train_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    ])