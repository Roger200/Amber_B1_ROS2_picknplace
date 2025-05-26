cmake_minimum_required(VERSION 3.8)
project(gazebo_ros_link_attacher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED) # Gazebo libraries
find_package(rosidl_default_generators REQUIRED) # For custom service generation

# Add the service definition
set(srv_files
  "srv/Attach.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

# Add the plugin library
add_library(${PROJECT_NAME} SHARED src/gazebo_ros_link_attacher_plugin.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS} # From find_package(gazebo_dev)
  ${IGNITION_MATH_INCLUDE_DIRS} # Gazebo's math library
)
target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES} # From find_package(gazebo_dev)
  ${IGNITION_MATH_LIBRARIES}
  # Link against other necessary libraries, e.g., SDF
  SDF::sdf # If directly using SDF API
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  gazebo_ros
  # Add other dependencies if needed
)

# Install plugin library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory (if you have one)
# install(DIRECTORY include/
#   DESTINATION include
# )

# Install the srv files and generated code
install(
    DIRECTORY srv
    DESTINATION share/${PROJECT_NAME}/srv
)
rosidl_install_interfaces(
    TARGETS rosidl_target_${PROJECT_NAME}
    DESTINATION share/${PROJECT_NAME}/srv
)


# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
