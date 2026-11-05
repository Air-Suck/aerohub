cmake_minimum_required(VERSION 3.16)
project(mrs_formation_reshaping)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryMsg.msg" "srv/SrvGetReshapingTrajectories.srv"
  DEPENDENCIES geometry_msgs)
add_library(FormationReshaper src/formation_reshaper.cpp)
target_include_directories(FormationReshaper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(FormationReshaper Eigen3::Eigen)
ament_target_dependencies(FormationReshaper rclcpp)
add_executable(cat_ora_node src/node.cpp)
target_link_libraries(cat_ora_node FormationReshaper)
ament_target_dependencies(cat_ora_node rclcpp geometry_msgs)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(cat_ora_node "${typesupport}")
install(TARGETS FormationReshaper cat_ora_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/ros2_proxy.py DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
