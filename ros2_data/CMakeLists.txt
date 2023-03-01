cmake_minimum_required(VERSION 3.5)
project(ros2_data)

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

# ADD ROS2.0 .msg, .srv and .action files:
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointPose.msg"
  "msg/JointPoseS.msg"
  "action/MoveJ.action"
  "action/MoveJs.action"
  "action/MoveG.action"
  "action/MoveXYZW.action"
  "action/MoveL.action"
  "action/MoveR.action"
  "action/MoveXYZ.action"
  "action/MoveYPR.action"
  "action/MoveROT.action"
  "action/MoveRP.action"
)
  
# IF ANY SCRIPT TO BE INSTALLED, add here:
#install(PROGRAMS  
#  ADD SCRIPTS HERE
#  DESTINATION lib/${PROJECT_NAME} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)AddTwoInts
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()