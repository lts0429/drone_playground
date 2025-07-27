import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    #endregion

    return LaunchDescription(nodes)
