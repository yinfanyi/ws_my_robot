cmake_minimum_required(VERSION 3.22)
project(moveit_servo)

install(
  TARGETS
    servo_keyboard_input
    servo_cpp_interface_demo
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)
#find_package(rosbag REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  rviz_visual_tools
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
  moveit_servo
  moveit_task_constructor_core
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(doc/examples/realtime_servo)


ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(include)

add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies(servo_keyboard_input  ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(servo_cpp_interface_demo src/servo_cpp_interface_demo.cpp)
target_include_directories(servo_cpp_interface_demo PUBLIC include)
ament_target_dependencies(servo_cpp_interface_demo  ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add executable for using a controller
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/teleop_demo/joystick_servo_example.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "my_robot_servo::JoyToServoPub")

ament_package()