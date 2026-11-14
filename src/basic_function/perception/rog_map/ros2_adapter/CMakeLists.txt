cmake_minimum_required(VERSION 3.8)
project(uav_rog_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rog_map REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uav_rog_map_interfaces REQUIRED)

add_executable(rog_map_node src/rog_map_node.cpp)
target_include_directories(rog_map_node PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(rog_map_node PRIVATE ORIGIN_AT_CORNER ROOT_DIR="")
ament_target_dependencies(rog_map_node
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  rog_map
  sensor_msgs
  tf2_ros
  uav_rog_map_interfaces
)
target_link_libraries(rog_map_node ${PCL_LIBRARIES})

install(TARGETS rog_map_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_test(NAME rog_map_ros_contract COMMAND ${CMAKE_COMMAND} -E env
    ROG_MAP_EXECUTABLE=$<TARGET_FILE:rog_map_node>
    ROG_MAP_TEST_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/test/test.yaml
    ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/smoke_ros2.py)
  set_tests_properties(rog_map_ros_contract PROPERTIES TIMEOUT 60)
endif()

ament_package()
