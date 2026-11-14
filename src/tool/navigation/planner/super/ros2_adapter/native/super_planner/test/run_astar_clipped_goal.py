#!/usr/bin/env python3
"""Run real A* on a reproducible empty local map without a simulator."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


def main():
    binary = Path(sys.argv[1]).resolve()
    source = Path(__file__).resolve().parents[1] / 'config/click_smooth_ros2.yaml'
    config = yaml.safe_load(source.read_text())
    config['astar']['map_voxel_num'] = [200, 200, 80]
    config['astar']['visual_process'] = False
    config['astar']['debug_visualization_en'] = False
    config['rog_map'].update(
        resolution=.2, inflation_resolution=.2, map_size=[60, 60, 16],
        fix_map_origin=[0, 0, 2], virtual_ground_height=-2,
        virtual_ceil_height=11, load_pcd_en=False,
    )
    config['rog_map']['ros_callback']['enable'] = False
    config['rog_map']['visualization']['enable'] = False
    config['rog_map']['raycasting']['enable'] = False
    # Separate DDS domain and logs; no PX4, Agent, AirSim or control publishers.
    with tempfile.TemporaryDirectory(prefix='super-astar-test-') as directory:
        path = Path(directory) / 'planner.yaml'
        path.write_text(yaml.safe_dump(config))
        env = dict(os.environ, ROS_DOMAIN_ID='191', ROS_LOCALHOST_ONLY='1',
                   ROS_LOG_DIR=str(Path(directory) / 'ros-logs'))
        env.pop('SUPER_CENTER_Z_MIN', None)
        env.pop('SUPER_CENTER_Z_MAX', None)
        result = subprocess.run([str(binary), str(path)], env=env, timeout=60)
        raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
