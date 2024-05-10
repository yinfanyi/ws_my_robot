from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(robot_name="my_arm1", package_name="my_arm1_moveit_config").to_moveit_configs()

    tutorial_node = Node(
        package="simple_demo",
        executable="get_robot_state",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    # print(moveit_config.robot_description_semantic)
    # print('--------------------------')
    # print(moveit_config.robot_description_kinematics)
    # print('---------------------------')
    return LaunchDescription([tutorial_node])