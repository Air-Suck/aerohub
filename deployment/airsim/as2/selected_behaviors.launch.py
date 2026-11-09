"""Reuse upstream behavior launch machinery for an explicitly selected subset."""
import importlib.util
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    names=os.environ.get('AS2_BEHAVIORS','takeoff,land').split(',')
    allowed={'takeoff','land','go_to','follow_path'}
    if not names or len(names)!=len(set(names)) or not set(names)<=allowed:
        raise ValueError('AS2_BEHAVIORS must list unique supported motion behaviors')
    source=Path(get_package_share_directory('as2_behaviors_motion'))/'launch/motion_behaviors_launch.py'
    spec=importlib.util.spec_from_file_location('as2_motion_behavior_launch',source)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions=[DeclareLaunchArgument('log_level',default_value='info'),
             DeclareLaunchArgument('namespace'),DeclareLaunchArgument('use_sim_time',default_value='false'),
             DeclareLaunchArgument('config_file',default_value='')]
    for name in names:actions.extend(module.get_behavior_launch_description(name))
    return LaunchDescription(actions)
