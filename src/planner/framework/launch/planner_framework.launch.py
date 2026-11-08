from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("uav_planner_framework"))
    return LaunchDescription([
        Node(
            package="uav_planner_framework",
            executable="planner_framework_node",
            name="src/planner/framework",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "registry_root": str(share / "registry"),
                "resolved_dir": "/tmp/uav_planner/resolved",
                "sample_rate_hz": 50.0,
            }],
        ),
    ])
