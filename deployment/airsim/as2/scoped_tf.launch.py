"""Apply per-vehicle TF topic remaps while retaining upstream AS2 launch defaults."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,GroupAction,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    def include(context):
        package=LaunchConfiguration('target_package').perform(context)
        name=LaunchConfiguration('target_launch').perform(context)
        arguments=json.loads(LaunchConfiguration('target_arguments').perform(context))
        namespace=arguments['namespace'].strip('/')
        path=Path(name) if not package else Path(get_package_share_directory(package))/'launch'/name
        return [GroupAction(actions=[SetRemap(src='/tf',dst='/'+namespace+'/tf'),
            SetRemap(src='/tf_static',dst='/'+namespace+'/tf_static'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=arguments.items())])]
    return LaunchDescription([DeclareLaunchArgument('target_package',default_value=''),
        DeclareLaunchArgument('target_launch'),DeclareLaunchArgument('target_arguments'),OpaqueFunction(function=include)])
