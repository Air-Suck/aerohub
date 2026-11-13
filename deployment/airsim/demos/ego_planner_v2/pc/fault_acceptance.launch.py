"""Isolated test launch: production native nodes and adapter, real DDS gates."""
import importlib.util
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    path='/input/src/planner/ego_planner_v2/ros2_adapter/bridge/launch/ego_as2.launch.py'
    spec=importlib.util.spec_from_file_location('production_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=module.nodes(context)
    result[-1]=Node(package='uav_ego_planner_adapter',executable='ego_as2_adapter',
        parameters=[{'resolved_config':LaunchConfiguration('resolved_config').perform(context),
                     'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
        remappings=[('/drone0/navigation/odom','/acceptance/drone0/odom'),
                    ('/drone0/navigation/points','/acceptance/drone0/points')],output='screen')
    result.append(ExecuteProcess(cmd=['python3','/input/deployment/airsim/demos/ego_planner_v2/pc/fault_injector.py'],output='screen'))
    return result


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('resolved_config'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),OpaqueFunction(function=nodes)])
