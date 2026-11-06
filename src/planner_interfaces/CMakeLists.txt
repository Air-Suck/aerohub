cmake_minimum_required(VERSION 3.8)
project(uav_planner_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowTimedPath.action"
  "msg/TimedPath.msg"
  "msg/BehaviorResult.msg"
  "msg/FleetSetpoint.msg"
  "msg/PlannerFailure.msg"
  "msg/TimedTrajectoryPoint.msg"
  "msg/TimedTrajectorySegment.msg"
  "msg/VehicleResult.msg"
  "msg/VehicleSetpoint.msg"
  "msg/VehicleTrajectory.msg"
  "srv/ControlPlanner.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
