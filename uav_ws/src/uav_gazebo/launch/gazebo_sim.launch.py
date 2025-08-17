from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart
import os
import xacro
import yaml

def generate_launch_description():

    nodes = []

    #region: Launch gazebo simulation
    world = DeclareLaunchArgument('world', default_value='empty', description='world to spawn in gazebo')
    world_path = PathJoinSubstitution([
        FindPackageShare('uav_gazebo'),
        'world',
        PythonExpression(["'", LaunchConfiguration("world"), "'", " + '.sdf'"])
    ])
    
    gazebo_config = PathJoinSubstitution([FindPackageShare('uav_gazebo'), 'config', 'standard.config'])
    
    gazebo_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch'),
        '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r ', world_path, ' --gui-config ', gazebo_config]}.items()
    )

    nodes += [world, gazebo_node]
    #endregion

    #region: Static transform from base_link to camera_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_tf',
        arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )
    nodes.append(static_tf_node)
    #endregion
        
    return LaunchDescription(nodes)
