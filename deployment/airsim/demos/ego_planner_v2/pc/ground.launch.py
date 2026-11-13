"""Two-vehicle AS2 capability startup only: no mission, goals or arming calls."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    actions = []
    config = '/opt/onboard_computer/config.yaml'
    fleet = yaml.safe_load(Path('/input/deployment/airsim/demos/ego_planner_v2/pc/fleet.yaml').read_text())
    for ns, vehicle in fleet['vehicles'].items():
        platform = Path('/tmp') / (ns + '-platform.yaml')
        platform.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'use_sim_time': True, 'cmd_freq': 50.0, 'info_freq': 10.0,
            'external_odom': False, **vehicle}}}))
        launches = [
            ('as2_platform_pixhawk', 'pixhawk_launch.py', {'platform_config_file': str(platform)}),
            ('as2_state_estimator', 'state_estimator_launch.py', {
                'config_file': config, 'plugin_name': 'raw_odometry',
                'plugin_config_file': config, 'use_sim_time': 'true'}),
            ('as2_motion_controller', 'controller_launch.py', {
                'config_file': config, 'plugin_name': 'pid_speed_controller',
                'plugin_config_file': '/opt/onboard_computer/pid_speed_controller.yaml',
                'use_sim_time': 'true'}),
            ('uav_follow_path_execution', 'motion_behaviors.launch.py', {
                'config_file': config, 'use_sim_time': 'true'}),
        ]
        for package, name, args in launches:
            path = Path(get_package_share_directory(package)) / 'launch' / name
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                launch_arguments={'namespace': ns, **args}.items()))
        actions.append(Node(package='as2_navigation_inputs', executable='navigation_inputs_node',
            namespace=ns, output='screen', parameters=[{
                'use_sim_time': True, 'target_frame': ns + '/odom',
                'body_frame': ns + '/base_link',
                'odom_topic': 'sensor_measurements/odom',
                'cloud_topic': 'sensor_measurements/Lidar/points',
                'transform_wait': 0.25, 'max_input_age': 1.0, 'queue_size': 20}]))
    return LaunchDescription(actions)
