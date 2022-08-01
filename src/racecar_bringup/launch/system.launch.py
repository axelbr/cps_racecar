import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    bringup_pkg = FindPackageShare(package='racecar_bringup').find('racecar_bringup')
    description_pkg = FindPackageShare(package='racecar_description').find('racecar_description')

    sensor_config = os.path.join(bringup_pkg, 'config', 'sensors.yaml')
    controller_config = os.path.join(bringup_pkg, 'config', 'controllers.yaml')
    teleop_config = os.path.join(bringup_pkg, 'config', 'teleop.yaml')
    urdf_model_path = os.path.join(description_pkg, 'urdf', 'racecar.urdf.xacro')
  
    description_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(description_pkg, 'launch', 'description.launch.py')),
        launch_arguments={
            'model': urdf_model_path
        }.items()
    )

    sensors_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'sensors.launch.py')),
        launch_arguments={
            'sensor_config': sensor_config
        }.items()
    )

    controllers_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'controllers.launch.py')),
        launch_arguments={
            'controller_config': controller_config,
            'model': urdf_model_path
        }.items()
    )

    teleop_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(bringup_pkg, 'launch', 'teleop.launch.py')),
        launch_arguments={
            'teleop_config': teleop_config
        }.items()
    )

    return LaunchDescription([
        description_launch,
        sensors_launch,
        controllers_launch,
        teleop_launch
    ])
