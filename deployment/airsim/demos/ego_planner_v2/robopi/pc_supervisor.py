"""Keep the pinned flight-board supervisor; assign DDS profiles by child role."""
import importlib.util
import sys

spec = importlib.util.spec_from_file_location('flight_board_supervisor', '/opt/flight_board/supervisor.py')
supervisor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(supervisor)
original_commands = supervisor.commands


def commands(mode, env):
    rows = original_commands(mode, env)
    roles = ['clock'] if mode == 'clock' else ['agent', None, 'sensors']
    for (args, environment, cwd), role in zip(rows, roles):
        if role:
            environment = dict(environment, FASTRTPS_DEFAULT_PROFILES_FILE='/session/pc-dds-'+role+'.xml')
        else:
            # PX4's default inline XML overrides the Agent's transport defaults.
            # Select its native named-profile path instead (px4_participant).
            environment = dict(environment, PX4_PARAM_UXRCE_DDS_PTCFG='2')
        yield args, environment, cwd


supervisor.commands = commands
if __name__ == '__main__':
    sys.exit(supervisor.main())
