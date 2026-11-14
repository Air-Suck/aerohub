from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="drone0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("plugin_name", default_value="dpv_slam"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uav_dpv_slam"), "config", "estimator.yaml"]),
        ),
        DeclareLaunchArgument(
            "plugin_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uav_dpv_slam"), "config", "estimator.yaml"]),
        ),
        Node(
            package="as2_state_estimator",
            executable="as2_state_estimator_node",
            namespace=LaunchConfiguration("namespace"),
            name="state_estimator",
            parameters=[
                LaunchConfiguration("config_file"),
                LaunchConfiguration("plugin_config_file"),
                {
                    "plugin_name": LaunchConfiguration("plugin_name"),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
            output="screen",
        ),
    ])
