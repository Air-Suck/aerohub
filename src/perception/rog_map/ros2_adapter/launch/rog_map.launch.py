from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("uav_rog_map"))
    default_config = str(share / "config" / "airsim.yaml")
    arguments = [
        DeclareLaunchArgument("namespace", default_value="drone0"),
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("global_frame", default_value="drone0/odom"),
        DeclareLaunchArgument("sensor_frame", default_value="drone0/Lidar"),
        DeclareLaunchArgument("cloud_topic", default_value="navigation/points"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]
    node = Node(
        package="uav_rog_map",
        executable="rog_map_node",
        namespace=LaunchConfiguration("namespace"),
        name="rog_map",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "config_path": LaunchConfiguration("config"),
            "global_frame": LaunchConfiguration("global_frame"),
            "sensor_frame": LaunchConfiguration("sensor_frame"),
            "cloud_topic": LaunchConfiguration("cloud_topic"),
        }],
    )
    return LaunchDescription(arguments + [node])
