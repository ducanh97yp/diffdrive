cmake_minimum_required(VERSION 3.8)
project(lio_sam_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(gz_lidar_to_ouster src/gz_lidar_to_ouster.cpp)
ament_target_dependencies(gz_lidar_to_ouster
  rclcpp sensor_msgs pcl_ros pcl_conversions)

add_executable(pcd_map_localizer src/pcd_map_localizer.cpp)
ament_target_dependencies(pcd_map_localizer
  rclcpp sensor_msgs geometry_msgs pcl_ros pcl_conversions tf2 tf2_ros tf2_eigen tf2_geometry_msgs)

install(TARGETS
  gz_lidar_to_ouster
  pcd_map_localizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
