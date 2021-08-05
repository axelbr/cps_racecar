import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription, condition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar_navigation')
    bringup_dir = get_package_share_directory('nav2_bringup')


    with_rviz = LaunchConfiguration('with_rviz')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    declare_rviz_cmd = DeclareLaunchArgument(
        name='with_rviz', 
        default_value='False', 
        description='Whether to run RViZ'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=os.path.join(pkg_dir, 'maps', 'room.yaml'),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(pkg_dir, 'params', 'nav.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    rviz_node =  Node(
          package='rviz2', 
          executable='rviz2', 
          name="rviz2", 
          output='screen', 
          arguments=[f'-d {pkg_dir}/rviz/navigation.rviz'],
          condition=IfCondition(with_rviz)
    )

    bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_namespace': use_namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'map': map_yaml_file,
                              'slam': slam
                              }.items()),
                        
    ld = LaunchDescription()
    ld.add_action(rviz_node)
    ld.add_action(bringup_launch)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    return ld