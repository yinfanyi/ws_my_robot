import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)


def generate_launch_description():
    package_name = 'my_robot_description'
    urdf_name = "my_robot.urdf"

    moveit_config = MoveItConfigsBuilder("multipleshapes", package_name="my_arm1_moveit_config").to_moveit_configs()

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    # )

    # Publish TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description],
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
        )
    
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]
    
    pkg_share1 = FindPackageShare(package='simple_demo').find('simple_demo')
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value= os.path.join(pkg_share1, "rviz/moveit.rviz"),
        )
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", LaunchConfiguration("rviz_config")],
        # parameters=rviz_parameters,
        )
    
    # ld.add_action(static_tf)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld