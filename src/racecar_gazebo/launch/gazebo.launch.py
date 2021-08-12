import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_description = launch_ros.substitutions.FindPackageShare(package='racecar_description').find('racecar_description')
    pkg_gazebo = launch_ros.substitutions.FindPackageShare(package='racecar_gazebo').find('racecar_gazebo')
    default_model_path = os.path.join(pkg_description, 'urdf/racecar.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz/urdf_config.rviz')
    world_path=os.path.join(pkg_gazebo, 'worlds/silverstone_track.sdf')
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'racecar', '-topic', 'robot_description'],
        output='screen'
    )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
       
    ])