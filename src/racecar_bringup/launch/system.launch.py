from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():

    pkg_share = FindPackageShare(package='racecar_bringup').find('racecar_bringup')
    description_pkg = FindPackageShare(package='racecar_description').find('racecar_description')
    default_model_path = os.path.join(description_pkg, 'urdf/racecar.urdf.xacro')
    controllers_file = os.path.join(description_pkg, 'controllers', 'ros2_control_controllers.yaml')

    system_config = os.path.join(
        pkg_share,
        'config',
        'racecar.yaml'
    )
   
    lidar_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('system_config')]
    )

    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            os.path.join(pkg_share, 'config', 'default_controllers.yaml')
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )


    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "-c", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    description_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(description_pkg, 'launch/description.launch.py'))
    )
                        


    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path, description='Path to urdf file.'),
        DeclareLaunchArgument('system_config', default_value=system_config, description='System configuration file.'),
        lidar_node,
        controller_manager,
        description_launch,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    ])