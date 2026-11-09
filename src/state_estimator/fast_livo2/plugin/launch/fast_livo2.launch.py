from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    cloud_frame = [namespace, '/livo_world']
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port', default_value='18761'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('uav_fast_livo2'), 'config', 'estimator.yaml'])),
        Node(package='uav_fast_livo2', executable='bridge.py', namespace=namespace,
             parameters=[{'use_sim_time': sim, 'forward_clock': sim, 'cloud_frame': cloud_frame,
                          'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}], output='screen'),
        Node(package='as2_state_estimator', executable='as2_state_estimator_node', namespace=namespace,
             parameters=[LaunchConfiguration('config'), {'use_sim_time': sim,
                          'plugin_name': 'fast_livo2', 'cloud_frame': cloud_frame}], output='screen'),
    ])
