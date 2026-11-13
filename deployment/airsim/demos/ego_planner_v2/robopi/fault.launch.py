"""Session-only fault wrapper around the installed, unchanged EGO capability."""
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROOT = Path(__file__).resolve().parent

def nodes(context):
    path=Path(get_package_share_directory('uav_ego_planner_adapter'))/'launch/ego_as2.launch.py'
    spec=importlib.util.spec_from_file_location('installed_ego_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=module.nodes(context)
    vehicle=LaunchConfiguration('vehicle',default='').perform(context)
    result[-1]=Node(package='uav_ego_planner_adapter',executable='ego_as2_adapter',
        namespace=vehicle,
        parameters=[{'resolved_config':LaunchConfiguration('resolved_config').perform(context),
                     'vehicle':vehicle,
                     'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
        remappings=[('/drone0/navigation/odom','/acceptance/drone0/odom'),
                    ('/drone0/navigation/points','/acceptance/drone0/points')],output='screen')
    result.append(ExecuteProcess(cmd=['python3',str(ROOT/'board_fault_injector.py')],output='screen'))
    return result

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('resolved_config'),
        DeclareLaunchArgument('vehicle',default_value=''),
        DeclareLaunchArgument('use_sim_time',default_value='true'),OpaqueFunction(function=nodes)])
