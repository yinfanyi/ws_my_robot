# # from moveit_configs_utils import MoveItConfigsBuilder
# # from moveit_configs_utils.launches import generate_demo_launch


# # def generate_launch_description():
# #     moveit_config = MoveItConfigsBuilder("multipleshapes", package_name="my_arm1_moveit_config").to_moveit_configs()
# #     return generate_demo_launch(moveit_config)

# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch.conditions import IfCondition, UnlessCondition
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.actions import ExecuteProcess
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
# import yaml

# def load_file(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)

#     try:
#         with open(absolute_file_path, "r") as file:
#             return file.read()
#     except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
#         return None


# def load_yaml(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)

#     try:
#         with open(absolute_file_path, "r") as file:
#             return yaml.safe_load(file)
#     except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
#         return None


# def generate_launch_description():

#     declared_arguments = []
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "rviz_config",
#             default_value="moveit.rviz",
#             description="RViz configuration file",
#         )
#     )

#     return LaunchDescription(
#         declared_arguments + [OpaqueFunction(function=launch_setup)]
#     )


# def launch_setup(context, *args, **kwargs):

#     moveit_config = (
#         MoveItConfigsBuilder("multipleshapes", package_name="my_arm1_moveit_config")
#         .robot_description(os.path.join(get_package_share_directory("my_robot_description"), "urdf/my_robot.urdf"))
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .planning_scene_monitor(
#             publish_robot_description=True, publish_robot_description_semantic=True
#         )
#         .planning_pipelines(
#             pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
#         )
#         .to_moveit_configs()
#     )

#     # Start the actual move_group node/action server
#     run_move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[moveit_config.to_dict()],
#     )

#     rviz_base = LaunchConfiguration("rviz_config")
#     rviz_config = PathJoinSubstitution(
#         [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
#     )

#     # rviz_base = os.path.join(
#     #     get_package_share_directory("moveit_resources_prbt_moveit_config"), "launch"
#     # )
#     # rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

#     # RViz
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config],
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#             moveit_config.planning_pipelines,
#             moveit_config.joint_limits,
#         ],
#     )

#     # Static TF
#     static_tf = Node(
#         package="tf2_ros",
#         executable="static_transform_publisher",
#         name="static_transform_publisher",
#         output="log",
#         arguments=["0.0", "0.0", "0.0", "0.0", "world", "base_link"],
#     )

#     # Publish TF
#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         name="robot_state_publisher",
#         output="both",
#         parameters=[moveit_config.robot_description],
#     )

#     # ros2_control using FakeSystem as hardware
#     ros2_controllers_path = os.path.join(
#         get_package_share_directory("my_arm1_moveit_config"),
#         "config",
#         "ros2_controllers.yaml",
#     )
#     ros2_control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[moveit_config.robot_description, ros2_controllers_path],
#         output="both",
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=[
#             "joint_state_broadcaster",
#             "--controller-manager-timeout",
#             "300",
#             "--controller-manager",
#             "/controller_manager",
#         ],
#     )

#     arm_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["my_arm1_controller", "-c", "/controller_manager"],
#     )

#     hand_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["hand_controller", "-c", "/controller_manager"],
#     )
#     nodes_to_start = [
#         rviz_node,
#         static_tf,
#         robot_state_publisher,
#         run_move_group_node,
#         ros2_control_node,
#         joint_state_broadcaster_spawner,
#         arm_controller_spawner,
#         hand_controller_spawner,
#     ]

#     return nodes_to_start

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my_arm1_moveit_config").to_moveit_configs()

    tutorial_node = Node(
        package="moveit2_tutorials",
        executable="robot_model_and_robot_state_tutorial",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([tutorial_node])


