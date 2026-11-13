cmake_minimum_required(VERSION 3.8)
project(uav_fast_livo2_ros2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(pointcloud_to_livox src/pointcloud_to_livox.cpp)
ament_target_dependencies(pointcloud_to_livox
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

add_executable(sensor_qos_adapter src/sensor_qos_adapter.cpp)
ament_target_dependencies(sensor_qos_adapter
  rclcpp
  sensor_msgs
)

add_executable(output_adapter src/output_adapter.cpp)
ament_target_dependencies(output_adapter
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
)

install(TARGETS pointcloud_to_livox sensor_qos_adapter output_adapter DESTINATION lib/${PROJECT_NAME})
ament_package()
