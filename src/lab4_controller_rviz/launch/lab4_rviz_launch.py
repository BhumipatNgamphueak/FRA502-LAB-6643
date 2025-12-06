#!/usr/bin/env python3
"""
LAB4 Launch File - COMPLETE VERSION
Includes all nodes as specified in LAB4.pdf
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Launch all nodes required for LAB4:
    1. Robot State Publisher (for visualization)
    2. Mock Robot (simulates the actual robot)
    3. Random Pose Node (generates random targets for AM mode)
    4. Controller Node (implements IPK, TO, AM modes)
    5. RViz2 (visualization)
    """

    # Get URDF/xacro file
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("lab4_description"), "robot", "visual", "my-robot.xacro"]
    )

    # Process xacro to get robot description
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str
    )

    # Node 1: Robot State Publisher (publishes TF transforms)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "publish_frequency": 50.0}
        ],
        output="screen",
    )

    # Node 2: Mock Robot (simulates joint states)
    mock_robot = Node(
        package="lab4_controller_rviz",
        executable="mock_robot_node.py",
        name="mock_robot",
        parameters=[
            {
                "kp": 20.0,  # Position gain
                "kd": 5.0,  # Damping gain
                "max_velocity": 3.0,
                "max_acceleration": 10.0,
            }
        ],
        output="screen",
    )

    # Node 3: Random Pose Node (as specified in LAB4 for AM mode)
    random_pose = Node(
        package="lab4_controller_rviz",
        executable="random_pose_node.py",
        name="random_pose",
        output="screen",
    )

    # Node 4: Controller Node (main controller with 3 modes)
    controller = Node(
        package="lab4_controller_rviz",
        executable="controller_node_rviz.py",
        name="controller",
        parameters=[
            {
                "control_rate": 50.0,
                "singularity_threshold": 0.01,
                "max_joint_velocity": 3.0,
                "am_timeout": 10.0,
                "am_reached_tolerance": 0.05,  # 5cm
            }
        ],
        output="screen",
    )

    # Node 5: RViz2 for visualization
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("lab4_description"), "config", "display.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    ee_publisher = Node(
        package="lab4_controller_rviz",
        executable="end_effector_publisher.py",
        name="ee_publisher",
        output="screen",
    )

    return LaunchDescription(
        [
            # Launch all nodes
            robot_state_publisher,
            mock_robot,
            random_pose,  # The random_pose node as specified in LAB4
            controller,
            rviz,
            ee_publisher,
        ]
    )
