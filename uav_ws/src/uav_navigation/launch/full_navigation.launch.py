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
    #endregion
    
    #region: nodes and launches
            
    #region: localization
    localization_params_file = os.path.join(get_package_share_directory("uav_navigation"), 'params', 'localization.yaml')    
    ekf_node = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[localization_params_file, {'use_sim_time': sim}],
            remappings=[("odometry/filtered", "odometry/local")],
        )
    nodes += [ekf_node]
    
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )    
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': localization_params_file
        }.items()
    )
    nodes += [slam_node]           
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
    nodes += [nav2_launch]
    #endregion
    
    return LaunchDescription(nodes)
