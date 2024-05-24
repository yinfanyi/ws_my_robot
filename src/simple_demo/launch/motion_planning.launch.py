import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_config = load_file(
        "my_robot_description", "urdf/my_robot.urdf"
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "my_arm1_moveit_config", "config/multipleshapes.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "my_arm1_moveit_config", "config/kinematics.yaml"
    )

    planning_yaml = load_yaml(
        "my_arm1_moveit_config", "config/ompl_planning.yaml"
    )

    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

    return LaunchDescription(
        [
            Node(
                package="simple_demo",
                executable="motion_planning",
                name="motion_planning",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    planning_yaml,
                    planning_plugin,
                ],
            )
        ]
    )
