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
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(custom_msg REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_visual_tools_INCLUDE_DIRS}
  ${moveit_ros_move_group_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometric_shapes_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS}
  ${custom_msg_INCLUDE_DIRS}
)



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
  custom_msg
  )

  add_executable(ArmMoverWithCollisionNode src/ArmMoverWithCollisionNode.cpp)
  ament_target_dependencies(
    ArmMoverWithCollisionNode
    rclcpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_ros_move_group
    moveit_msgs
    pluginlib
    tf2
    tf2_ros
    geometric_shapes
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
install(TARGETS 
ArmMoverWithCollisionNode
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
# ament_python_install_package(natnet)


# Install Python modules
install(DIRECTORY natnet/
  DESTINATION lib/${PROJECT_NAME}
)

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
