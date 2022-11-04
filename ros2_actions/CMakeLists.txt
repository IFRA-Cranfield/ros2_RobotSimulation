cmake_minimum_required(VERSION 3.5)
project(ros2_actions)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# REQUIRED to -> Launch move_group interface:
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# REQUIRED to -> Include ros2_data package:
find_package(ros2_data REQUIRED)

# REQUIRED to -> Execute ACTIONS:
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

# ===== ACTIONS ===== #
# moveJ_action.cpp:
add_executable(moveJ_action scripts/moveJ_action.cpp)
ament_target_dependencies(
  moveJ_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveJ_action
  DESTINATION lib/${PROJECT_NAME})
# moveJs_action.cpp:
add_executable(moveJs_action scripts/moveJs_action.cpp)
ament_target_dependencies(
  moveJs_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveJs_action
  DESTINATION lib/${PROJECT_NAME})
# moveG_action.cpp:
add_executable(moveG_action scripts/moveG_action.cpp)
ament_target_dependencies(
  moveG_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveG_action
  DESTINATION lib/${PROJECT_NAME})
# moveXYZW_action.cpp:
add_executable(moveXYZW_action scripts/moveXYZW_action.cpp)
ament_target_dependencies(
  moveXYZW_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveXYZW_action
  DESTINATION lib/${PROJECT_NAME})
# moveL_action.cpp:
add_executable(moveL_action scripts/moveL_action.cpp)
ament_target_dependencies(
  moveL_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveL_action
  DESTINATION lib/${PROJECT_NAME})
# moveR_action.cpp:
add_executable(moveR_action scripts/moveR_action.cpp)
ament_target_dependencies(
  moveR_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveR_action
  DESTINATION lib/${PROJECT_NAME})
# moveRs_action.cpp:
add_executable(moveRs_action scripts/moveRs_action.cpp)
ament_target_dependencies(
  moveRs_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveRs_action
  DESTINATION lib/${PROJECT_NAME})
# moveXYZ_action.cpp:
add_executable(moveXYZ_action scripts/moveXYZ_action.cpp)
ament_target_dependencies(
  moveXYZ_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveXYZ_action
  DESTINATION lib/${PROJECT_NAME})
# moveYPR_action.cpp:
add_executable(moveYPR_action scripts/moveYPR_action.cpp)
ament_target_dependencies(
  moveYPR_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveYPR_action
  DESTINATION lib/${PROJECT_NAME})
# moveROT_action.cpp:
add_executable(moveROT_action scripts/moveROT_action.cpp)
ament_target_dependencies(
  moveROT_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveROT_action
  DESTINATION lib/${PROJECT_NAME})
# moveRP_action.cpp:
add_executable(moveRP_action scripts/moveRP_action.cpp)
ament_target_dependencies(
  moveRP_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  ros2_data)
install(TARGETS
  moveRP_action
  DESTINATION lib/${PROJECT_NAME})

ament_package()
