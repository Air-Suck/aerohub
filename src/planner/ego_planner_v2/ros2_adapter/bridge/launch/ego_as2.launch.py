from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from uav_ego_planner_adapter.scope import AdapterScope

def nodes(context):
    path = LaunchConfiguration('resolved_config').perform(context)
    config = yaml.safe_load(Path(path).read_text())
    # Default also supports the existing session-only fault wrapper, which
    # invokes nodes(context) without declaring a vehicle launch argument.
    vehicle = LaunchConfiguration('vehicle', default='').perform(context)
    scope = AdapterScope(config, vehicle)
    parameters_path = config.get('native_parameters_file',str(Path(get_package_share_directory('ego_planner'))/'config/ego.yaml'))
    parameters = yaml.safe_load(Path(parameters_path).read_text())['/**']['ros__parameters']
    for limit,key in [('velocity','max_vel'),('acceleration','max_acc'),('tolerance','feasibility_tolerance')]:
        if parameters['manager.'+key] != config['limits'][limit]:
            raise ValueError('planner/adapter limit mismatch: '+limit)
    if parameters['optimization.max_vel'] != config['limits']['velocity'] or parameters['optimization.max_acc'] != config['limits']['acceleration']:
        raise ValueError('optimizer/manager limits differ')
    result = []
    for owned in scope.config['vehicles']:
        identity = config['planner_ids'][owned]
        if config['deployment'][owned]['namespace'] != f'drone{identity}':
            raise ValueError('native launch requires namespace drone<planner_id>')
        result.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ego_planner'))/'launch/ego.launch.py')),
            launch_arguments={'drone_id':str(identity),'odom_topic':scope.native(owned,'odom'),
                'cloud_topic':scope.native(owned,'points'),'goal_topic':scope.native(owned,'goal'),
                'odom_twist_in_body_frame':'true',
                'use_sim_time':LaunchConfiguration('use_sim_time'),
                'params_file':parameters_path}.items()))
    result.append(Node(package='uav_ego_planner_adapter', executable='ego_as2_adapter',
                       namespace=scope.namespace,
                       parameters=[{'resolved_config':path,'vehicle':vehicle,
                                    'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}], output='screen'))
    return result

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('resolved_config'),
                              DeclareLaunchArgument('vehicle',default_value=''),
                              DeclareLaunchArgument('use_sim_time',default_value='false'),
                              OpaqueFunction(function=nodes)])
