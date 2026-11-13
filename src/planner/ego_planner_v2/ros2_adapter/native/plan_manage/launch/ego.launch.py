from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("odom_twist_in_body_frame", default_value="false"),
        DeclareLaunchArgument("odom_topic", default_value="/drone0/navigation/odom"),
        DeclareLaunchArgument("cloud_topic", default_value="/drone0/navigation/points"),
        DeclareLaunchArgument("goal_topic", default_value="/goal_with_id"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([
            FindPackageShare("ego_planner"), "config", "ego.yaml"])),
    ]
    planner = Node(
        package="ego_planner", executable="ego_planner_node", name="ego_planner",
        namespace=["drone", LaunchConfiguration("drone_id")], output="screen",
        parameters=[LaunchConfiguration("params_file"), {
            "manager.drone_id": ParameterValue(LaunchConfiguration("drone_id"), value_type=int),
            "fsm.odom_twist_in_body_frame": ParameterValue(LaunchConfiguration("odom_twist_in_body_frame"), value_type=bool),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
        remappings=[("~/odom_world", LaunchConfiguration("odom_topic")),
                    ("~/grid_map/odom", LaunchConfiguration("odom_topic")),
                    ("~/grid_map/cloud", LaunchConfiguration("cloud_topic")),
                    ("/goal", LaunchConfiguration("goal_topic")),
                    ("~/planning/broadcast_traj_send", "/ego_swarm/trajectory"),
                    ("~/planning/broadcast_traj_recv", "/ego_swarm/trajectory")])
    return LaunchDescription(arguments + [planner])
