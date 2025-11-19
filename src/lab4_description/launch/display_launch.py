#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Use simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package shares
    desc_pkg = get_package_share_directory('lab4_description')
    sim_pkg = get_package_share_directory('lab4_simulation')

    # Get the parent directory (install/share) to make package:// URIs work
    install_share_dir = os.path.dirname(desc_pkg)
    
    # Set Gazebo resource paths
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_share_dir + ':' + 
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_share_dir + ':' + 
              os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    # Robot description (xacro)
    xacro_file = PathJoinSubstitution([
        FindPackageShare('lab4_description'), 
        'urdf',
        'my-robot_gazebo.xacro'
    ])
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time
    ])
    
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # Gazebo simulation launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Robot state publisher
    state_pub = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        name='robot_state_publisher', 
        output='screen',
        parameters=[
            {'robot_description': robot_description_param},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create', 
        name='spawn_entity', 
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'lab4_robot',
            '-allow_renaming', 'true',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.0'
        ]
    )

    # Controller manager spawners
    jsb_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    position_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_position_controller',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ROS <-> Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz (optional - will only launch if config exists)
    rviz_config = os.path.join(sim_pkg, 'rviz', 'lab4.rviz')
    rviz = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz', 
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        gz_model_path,
        ign_model_path,
        gz_sim,
        state_pub,
        spawn_entity,
        bridge,
        
        # After spawning the robot, start the joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb_spawner]
            )
        ),
        
        # After joint_state_broadcaster is active, start the position controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[position_spawner]
            )
        ),
        
        rviz
    ])