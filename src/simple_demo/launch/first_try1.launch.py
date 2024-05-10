import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from moveit_configs_utils.launches import generate_demo_launch



def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("multipleshapes", package_name="my_arm1_moveit_config").to_moveit_configs()
    ld:LaunchDescription = generate_demo_launch(moveit_config)
    execute_node = Node(
        package="simple_demo",
        executable="first_try",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    ld.add_action(execute_node)
    return ld
