from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('uav_follow_reference_acro'),
                          'config', 'external_acro.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('solve_timeout', default_value='0.1'),
        DeclareLaunchArgument('control_mode', default_value='body_rates'),
        DeclareLaunchArgument('output_enabled', default_value='false'),
        Node(package='uav_follow_reference_acro', executable='follow_reference_acro_node',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[config, {
                 'external_acro.solve_timeout': ParameterValue(LaunchConfiguration('solve_timeout'), value_type=float),
                 'external_acro.control_mode': LaunchConfiguration('control_mode'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'external_acro.output_enabled': ParameterValue(
                     LaunchConfiguration('output_enabled'), value_type=bool),
             }]),
    ])
