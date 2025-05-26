cmake_minimum_required(VERSION 3.8)
project(amber_pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_cpp REQUIRED) # Uncomment if using MoveItCpp API
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(gazebo_ros_link_attacher REQUIRED) # For Attach.srv


# Add executable
add_executable(pick_and_place_node src/pick_and_place_node.cpp)

ament_target_dependencies(pick_and_place_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  # moveit_cpp # Uncomment if using MoveItCpp API
  tf2_ros
  tf2_geometry_msgs
  moveit_visual_tools
  moveit_msgs
  gazebo_ros_link_attacher # For Attach.srv
)

# Install executable
install(TARGETS
  pick_and_place_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
