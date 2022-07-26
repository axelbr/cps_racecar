from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():

    pkg_share = FindPackageShare(package='racecar_bringup').find('racecar_bringup')
    description_pkg = FindPackageShare(package='racecar_description').find('racecar_description')

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

    description_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(description_pkg, 'launch/description.launch.py'))
    )
                        


    return LaunchDescription([
        DeclareLaunchArgument('system_config', default_value=system_config, description='System configuration file.'),
        lidar_node,
        description_launch
    ])