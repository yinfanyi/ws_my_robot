import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    package_name = 'my_robot_description'
    urdf_name = "my_robot.urdf"
    moveit_config = MoveItConfigsBuilder("multipleshapes", package_name="my_arm1_moveit_config").to_moveit_configs()
    
    ld = LaunchDescription()
    
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    
    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # urdf_file_name = 'r2d2.urdf.xml'
    # urdf = os.path.join(
    #     get_package_share_directory('urdf_tutorial_r2d2'),
    #     urdf_file_name)
    # with open(urdf_model_path, 'r') as infp:
    #     robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description],
        output='screen',
        # parameters=[{'use_sim_time': use_sim_time, 'robot_description': moveit_config.robot_description}],
        arguments=[urdf_model_path]
        )
    
    joint_state_publisher_node = Node(
        package='simple_demo2',
        executable='state_publisher',
        name='state_publisher',
        output='screen'
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
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        )
    
    # ld.add_action(static_tf)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)


    return ld
