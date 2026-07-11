cmake_minimum_required(VERSION 3.8)
project(lio_sam_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(gz_lidar_to_ouster src/gz_lidar_to_ouster.cpp)
ament_target_dependencies(gz_lidar_to_ouster
  rclcpp sensor_msgs pcl_ros pcl_conversions)

install(TARGETS
  gz_lidar_to_ouster
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
