cmake_minimum_required(VERSION 3.8)
project(depth_to_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

# Create the shared library for the component
add_library(depth_to_pointcloud_component SHARED src/depth_to_pointcloud_node.cpp)
target_include_directories(depth_to_pointcloud_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(depth_to_pointcloud_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(depth_to_pointcloud_component
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "sensor_msgs"
  "cv_bridge"
  "pcl_conversions"
  "message_filters"
)
# Add PCL include directories and libraries
target_include_directories(depth_to_pointcloud_component PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(depth_to_pointcloud_component ${PCL_LIBRARIES})

# Register the component
rclcpp_components_register_node(depth_to_pointcloud_component
  PLUGIN "depth_to_pointcloud::DepthToPointCloudNode"
  EXECUTABLE depth_to_pointcloud_node_exec
)

# Standalone executable to run the node easily without a container
add_executable(perception_node src/perception_node.cpp)
target_compile_features(perception_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(perception_node
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
)
# Link the standalone node against our component library
target_link_libraries(perception_node depth_to_pointcloud_component)

install(TARGETS 
  depth_to_pointcloud_component
  perception_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/ rviz/
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
