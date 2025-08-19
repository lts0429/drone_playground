import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    
    nodes = []
        
    #region: arguments
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='use simulation time if true, otherwise use real time'
    )
    sim = LaunchConfiguration('sim')
    nodes += [sim_arg]

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='palm_oil_plantation',
        description='world to spawn in gazebo'
    )
    # world_arg = DeclareLaunchArgument(
    #     'world',
    #     default_value='industrial-warehouse/industrial-warehouse',
    #     description='world to spawn in gazebo'
    # )
    world = LaunchConfiguration('world')
    nodes += [world_arg]
    #endregion
    
    #region: nodes and launches
            
    #region: gazebo (only simulation)   
    gazebo_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('uav_gazebo'), 'launch'),
        '/gazebo_sim.launch.py']),
        condition=IfCondition(sim)
    )
    nodes += [gazebo_node]
    #endregion
    
    # #region: navigation
    # navigation_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('uav_navigation'),
    #             'launch',
    #             'full_navigation.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'use_sim_time': sim
    #     }.items()
    # )
    # nodes += [navigation_node]
    # #endregion

    #region: rviz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('uav_bringup'),
        'config',
        'standard.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': sim}]
    )
    nodes += [rviz_node]
    #endregion

    return LaunchDescription(nodes)
