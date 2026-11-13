"""Use upstream launch helpers for other motions; replace only FollowPath."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    upstream = Path(get_package_share_directory("as2_behaviors_motion"))
    spec = importlib.util.spec_from_file_location(
        "upstream_motion_launch", upstream / "launch/motion_behaviors_launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = [
        DeclareLaunchArgument("namespace"),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("follow_path_plugin_name", default_value="follow_path_plugin_position"),
        DeclareLaunchArgument("timed_position_tolerance", default_value="0.25"),
        DeclareLaunchArgument("timed_continuity_tolerance", default_value="0.15"),
        DeclareLaunchArgument("timed_acceleration_continuity_tolerance", default_value="1.4"),
    ]
    for behavior in ("takeoff", "land", "go_to"):
        description += module.get_behavior_launch_description(behavior)
    description.append(Node(
        package="uav_follow_path_execution", executable="follow_path_extended_node",
        namespace=LaunchConfiguration("namespace"), output="screen",
        parameters=[str(upstream / "follow_path_behavior/config/config_default.yaml"), {
            "plugin_name": LaunchConfiguration("follow_path_plugin_name"),
            "vehicle_id": LaunchConfiguration("namespace"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "run_frequency": 50.0,
            "timed_position_tolerance": LaunchConfiguration("timed_position_tolerance"),
            "timed_continuity_tolerance": LaunchConfiguration("timed_continuity_tolerance"),
            "timed_acceleration_continuity_tolerance": LaunchConfiguration(
                "timed_acceleration_continuity_tolerance"),
        }],
    ))
    return LaunchDescription(description)
