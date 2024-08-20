cmake_minimum_required(VERSION 3.8)
project(collision_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Include Cpp "include" directory
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
# in order to use rclpy
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning REQUIRED)

# C++ executable
add_executable(CollisionCheckerNode src/CollisionCheckerNode.cpp)
ament_target_dependencies(
  CollisionCheckerNode
  rclcpp
  moveit_core
  moveit_ros_planning_interface 
  moveit_ros_planning
  geometry_msgs
  )
add_executable(CollisionObjUpdater src/CollisionObjUpdater.cpp)
ament_target_dependencies(
  CollisionObjUpdater
  rclcpp
  moveit_core
  moveit_ros_planning_interface 
  moveit_ros_planning
  geometry_msgs
  )
# Install C++ executables
install(TARGETS
CollisionCheckerNode
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
CollisionObjUpdater
  DESTINATION lib/${PROJECT_NAME}
)


# Python executable
# Install launch files.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# Install natnet 
install(DIRECTORY natnet/
  DESTINATION share/${PROJECT_NAME}/natnet
)
# Python executable
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(natnet)


# Install Python scripts
install(PROGRAMS
collision_pkg/OptiTrackPubNode.py
  DESTINATION lib/${PROJECT_NAME}
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
