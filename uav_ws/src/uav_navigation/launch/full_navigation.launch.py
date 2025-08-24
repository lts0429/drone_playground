import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.actions import TimerAction

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
    #endregion
    
    #region: nodes and launches
            
    #region: localization
    localization_params_file = os.path.join(get_package_share_directory("uav_navigation"), 'params', 'localization.yaml')    
    localization_launch_path = os.path.join(
        get_package_share_directory('uav_navigation'),
        'launch',
        'localization.launch.py'
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path),
        launch_arguments={
            'use_sim_time': sim,
            'params_file': localization_params_file,
        }.items()
    )
    nodes += [localization_launch]
    #endregion
    
    #region: nav2 stack
    nav2_open_field_params_file = PathJoinSubstitution([
        FindPackageShare('uav_navigation'),
        'params',
        'nav2.yaml'
    ])
    nav2_launch_path = os.path.join(
        get_package_share_directory('uav_navigation'),
        'launch',
        'nav2_launch.launch.py'
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': sim,
            'params_file': nav2_open_field_params_file,
        }.items()
    )
    nav2_launch_with_delay = TimerAction(
        period=5.0,  # delay in seconds
        actions=[nav2_launch]
    )
    nodes += [nav2_launch_with_delay]
    #endregion
    
    return LaunchDescription(nodes)
