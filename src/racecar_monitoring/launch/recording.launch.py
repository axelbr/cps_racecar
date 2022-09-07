from ast import arg
from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os

def generate_launch_description():

    pkg_share = FindPackageShare(package='racecar_monitoring').find('racecar_monitoring')
    default_rviz_config = os.path.join(pkg_share, 'rviz/default.rviz')


    rosbag_node = Node(
        package='rosbag2',
        executable='rosbag2',
        name='datacollector',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    plotter = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen',
        arguments=["-l", os.path.join(pkg_share, "config", "plots.xml")]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=conditions.IfCondition(LaunchConfiguration('rqt_gui'))
    )
   
   
    return LaunchDescription([
       DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config, description='Absolute path to rviz config file'),
       DeclareLaunchArgument(name='rqt_gui', default_value='False', description='Flag to enable joint_state_publisher_gui'),
       rviz_node,
       joint_state_publisher_gui_node,
       plotter
    ])