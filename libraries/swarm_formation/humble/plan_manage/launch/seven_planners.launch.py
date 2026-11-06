"""Seven native planners, with legacy fixtures or registered navigation inputs."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_nodes(context):
    share = Path(get_package_share_directory('ego_planner'))
    parameters = yaml.safe_load((share / 'config/humble.yaml').read_text())
    mode = LaunchConfiguration('input_mode').perform(context)
    if mode not in ('legacy', 'navigation'):
        raise ValueError('input_mode must be legacy or navigation')
    live = mode == 'navigation'
    simulation = LaunchConfiguration('use_sim_time').perform(context)
    if simulation not in ('auto', 'true', 'false'):
        raise ValueError('use_sim_time must be auto, true or false')
    use_sim_time = live if simulation == 'auto' else simulation == 'true'
    count = int(LaunchConfiguration("vehicle_count").perform(context))
    if not 3 <= count <= 7: raise ValueError("vehicle_count must be in [3,7]")
    owned = LaunchConfiguration("local_vehicle").perform(context)
    if owned and owned not in [f"drone{i}" for i in range(count)]: raise ValueError("invalid local_vehicle")
    config = LaunchConfiguration("parameters_file").perform(context)
    if config: parameters.update(yaml.safe_load(Path(config).read_text()))
    fleet = None
    if live:
        filename = LaunchConfiguration('fleet_config').perform(context)
        if not filename:
            raise ValueError('navigation mode requires fleet_config with measured fixed origins')
        fleet = yaml.safe_load(Path(filename).read_text())
        if set(fleet['vehicles']) != {f'drone{i}' for i in range(count)}:
            raise ValueError('navigation fleet must match configured vehicle_count; do not replicate inputs')
    frame = fleet.get('map_frame', 'swarm_map') if live else 'world'
    nodes = []
    for index in range(count):
        name = f'drone{index}'
        if owned and owned != name: continue
        overrides = {'manager.drone_id': index, 'use_sim_time': use_sim_time, 'optimization.vehicle_count': count}
        if live:
            entry = fleet['vehicles'][name]
            prefix = f'/swarm_formation/{name}'
            nodes.append(Node(package='plan_env', executable='navigation_bridge_node',
                name=f'{name}_swarm_inputs', namespace=f'/swarm_formation/{name}', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'source_frame': entry.get('source_frame', f'{name}/odom'),
                    'body_frame': entry.get('body_frame', f'{name}/base_link'),
                    'map_frame': frame, 'origin_enu': entry['origin_enu'],
                    'origin_yaw': float(entry.get('origin_yaw', 0.0)),
                    'max_input_age': float(fleet.get('input_timeout', 1.0)),
                    'odom_topic': entry.get('odom_topic', f'/{name}/navigation/odom'),
                    'cloud_topic': entry.get('cloud_topic', f'/{name}/navigation/points'),
                }]))
            odom_topic, cloud_topic = prefix+'/odom', prefix+'/points'
            overrides.update({'fsm.twist_in_body_frame': True, 'global_goal.altitude_from_message': True, 'grid_map.frame_id': frame,
                              'grid_map.input_timeout': float(fleet.get('input_timeout', 1.0))})
        else:
            odom_topic = f'/drone_{index}_visual_slam/odom'
            cloud_topic = '/map_generator/global_cloud'
        nodes.append(Node(
            package='ego_planner', executable='ego_planner_node',
            name=f'drone_{index}_ego_planner_node', output='screen',
            parameters=[dict(parameters, **overrides)],
            remappings=[
                ('~/odom_world', odom_topic), ('~/grid_map/odom', odom_topic),
                ('~/grid_map/cloud', cloud_topic),
                ('~/planning/trajectory', f'/drone_{index}_planning/trajectory'),
                ('~/planning/finish', f'/drone_{index}_planning/finish'),
                ('~/planning/broadcast_traj_send', '/swarm_formation/broadcast' if owned else '/broadcast_traj_from_planner'),
                ('~/planning/broadcast_traj_recv', '/swarm_formation/broadcast' if owned else '/broadcast_traj_to_planner'),
                ('/move_base_simple/goal', '/traj_start_trigger'),
            ]))
    if LaunchConfiguration('start_proxy').perform(context) == 'true':
      nodes.append(Node(package='ego_planner', executable='ros2_proxy.py', output='screen',
                      parameters=[{'use_sim_time': use_sim_time, 'publish_odometry': not live,
                                   'map_frame': frame, 'vehicle_count': count, 'relay_broadcast': not bool(owned)}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_count', default_value='7'),
        DeclareLaunchArgument('local_vehicle', default_value=''),
        DeclareLaunchArgument('parameters_file', default_value=''),
        DeclareLaunchArgument('start_proxy', default_value='true'),
        DeclareLaunchArgument('input_mode', default_value='legacy'),
        DeclareLaunchArgument('fleet_config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='auto'),
        OpaqueFunction(function=build_nodes),
    ])
