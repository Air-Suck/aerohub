cmake_minimum_required(VERSION 3.8)
project(uav_object_detection_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uav_inference_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectDetection.msg" "msg/ObjectDetectionFrame.msg" "srv/DetectObjects.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs uav_inference_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
