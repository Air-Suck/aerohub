"""Launch the AS2 state-estimator host with the external FAST-LIVO2 plugin."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file'),
        DeclareLaunchArgument('plugin_config_file'),
        # Keep the common launcher contract even though this external entry point owns the class.
        DeclareLaunchArgument('plugin_name', default_value='fast_livo2'),
        Node(
            package='as2_state_estimator',
            executable='as2_state_estimator_node',
            name='state_estimator',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[
                LaunchConfiguration('config_file'),
                LaunchConfiguration('plugin_config_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'plugin_name': 'fast_livo2',
                },
            ],
        ),
    ])
