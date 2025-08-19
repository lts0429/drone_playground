from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
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
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    nodes += [world, gazebo_node]
    #endregion
    
    #region: Bridge
    bridge_config = PathJoinSubstitution([FindPackageShare('uav_gazebo'), 'config', 'bridge_config.yaml'])
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{
            'config_file': ParameterValue(bridge_config, value_type=str)
        }]
    )
    nodes += [bridge_node]
    #endregion
    
    #region: Static transform from base_link to camera_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static_tf',
        arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )
    nodes.append(static_tf_node)
    
    left_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_camera_static_tf',
        arguments=['0.1', '-0.05', '0', '0', '0', '0', 'base_link', 'camera_left']
    )
    nodes.append(left_cam_tf)

    right_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_camera_static_tf',
        arguments=['0.1', '0.05', '0', '0', '0', '0', 'base_link', 'camera_right']
    )
    nodes.append(right_cam_tf)
    #endregion
        
    # # This command sets the follow offset
    # follow_cmd = [
    #     "ign", "service",
    #     "-s", "/gui/follow",
    #     "--reqtype", "ignition.msgs.StringMsg",
    #     "--reptype", "ignition.msgs.Boolean",
    #     "--timeout", "2000",
    #     "--req", "data: 'X3_custom'"
    # ]

    # # This command sets the follow offset
    # offset_cmd = [
    #     "ign", "service",
    #     "-s", "/gui/follow/offset",
    #     "--reqtype", "ignition.msgs.Vector3d",
    #     "--reptype", "ignition.msgs.Boolean",
    #     "--timeout", "2000",
    #     "--req", "x: -1.0, y: 0, z: 0.3"
    # ]
    # follow_service_node = LaunchDescription([
    #     # Delay execution by 10 seconds so Gazebo GUI is ready
    #     TimerAction(
    #         period=10.0,
    #         actions=[
    #             ExecuteProcess(cmd=follow_cmd, output="screen")
    #         ]
    #     ),
    #     # Call the offset service shortly after follow
    #     TimerAction(
    #         period=11.0,
    #         actions=[
    #             ExecuteProcess(cmd=offset_cmd, output="screen")
    #         ]
    #     )
    # ])
    # nodes += [follow_service_node]
    
    return LaunchDescription(nodes)
