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
    
    method_arg = DeclareLaunchArgument(
        'method',
        default_value='orb_slam3',
        description='localization method (options: orb_slam3)'
    )
    method = LaunchConfiguration('method')
    nodes += [method_arg]

    #endregion
         
    #region: localization 
    slam_pkg_path = get_package_share_directory("orbslam3")
    vocab_file = os.path.join(slam_pkg_path, "vocabulary", "ORBvoc.txt")
    settings_file = os.path.join(slam_pkg_path, "config", "stereo/gazebo.yaml")
    camera_type_arg = DeclareLaunchArgument(
        'camera_type', default_value='stereo', description='Camera type: mono, rgbd, stereo')
    nodes += [camera_type_arg]

    orb_slam_node = Node(
        package='orbslam3',
        executable=LaunchConfiguration('camera_type'),  # Get the executable based on camera type
        output='screen',
        parameters=[
            {"publish_tf": True},
        ],
        arguments=[
            vocab_file, 
            settings_file, 
            'False', 
            'True', 
            '--ros-args', '--log-level', 'info'
            ],
        remappings=[
            ('/camera/left', '/camera_left/image'),
            ('/camera/right', '/camera_right/image')
        ]
    )
    nodes += [orb_slam_node]
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom', '--ros-args', '--log-level', 'info']
    )
    nodes.append(static_tf_node)  
    
    # test_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='test_to_odom_static_tf',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link', '--ros-args', '--log-level', 'info']
    # )
    # nodes.append(test_tf_node)  
    #endregion
    
    return LaunchDescription(nodes)
