cmake_minimum_required(VERSION 3.8)
project(cube_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(detect_cube_node src/detect_cube_node.cpp)

# PCL dependencies
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Link PCL libraries
# Common PCL libraries: common, io, filters, segmentation, features, surface, kdtree, octree
# Add more as needed by your specific PCL usage
set(PCL_LIBRARIES
  pcl_common
  pcl_filters
  pcl_segmentation
  pcl_features
  pcl_surface # For ConvexHull or BoundingBox
  pcl_io      # If reading PCD files, etc.
  pcl_kdtree
  pcl_octree
  # Add other PCL components if used, e.g., pcl_visualization
)

ament_target_dependencies(detect_cube_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  PCL # This should bring in PCL_LIBRARIES if configured correctly by find_package(PCL)
  tf2_ros
  tf2_geometry_msgs
)

# Install executable
install(TARGETS
  detect_cube_node
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
