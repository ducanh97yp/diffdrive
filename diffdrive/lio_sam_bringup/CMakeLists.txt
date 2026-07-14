cmake_minimum_required(VERSION 3.8)
project(lio_sam_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

add_executable(gz_lidar_to_ouster src/gz_lidar_to_ouster.cpp)
ament_target_dependencies(gz_lidar_to_ouster
  rclcpp sensor_msgs pcl_ros pcl_conversions)

add_executable(pcd_map_localizer src/pcd_map_localizer.cpp)
ament_target_dependencies(pcd_map_localizer
  rclcpp sensor_msgs geometry_msgs nav_msgs pcl_ros pcl_conversions tf2 tf2_ros tf2_eigen tf2_geometry_msgs)

install(TARGETS
  gz_lidar_to_ouster
  pcd_map_localizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# nav2_costmap_2d::Layer plugin - see costmap_plugins.xml. Separate shared library,
# installed to plain lib/ (pluginlib .so search convention), not lib/${PROJECT_NAME}/
# where the two executables above go - no collision between the two.
add_library(elevation_traversability_layer SHARED
  src/elevation_traversability_layer.cpp)
target_include_directories(elevation_traversability_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(elevation_traversability_layer
  rclcpp pluginlib nav2_costmap_2d sensor_msgs nav_msgs tf2 tf2_ros tf2_eigen pcl_ros pcl_conversions)

install(TARGETS elevation_traversability_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(elevation_traversability_layer)
ament_export_dependencies(pluginlib nav2_costmap_2d sensor_msgs nav_msgs tf2 tf2_ros tf2_eigen)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_package()
