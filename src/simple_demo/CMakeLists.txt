cmake_minimum_required(VERSION 3.8)
project(simple_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(get_robot_state src/get_robot_state.cpp)
target_include_directories(get_robot_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(get_robot_state PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  get_robot_state
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS get_robot_state
  DESTINATION lib/${PROJECT_NAME})

add_executable(first_try src/first_try.cpp)
target_include_directories(first_try PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(first_try PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  first_try
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS first_try
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
