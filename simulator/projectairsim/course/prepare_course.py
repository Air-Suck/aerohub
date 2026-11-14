#!/usr/bin/env python3
"""Resolve a course YAML and the static robot visuals for UE's bundled Python."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--robot-config',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    cfg=yaml.safe_load(args.config.read_text())
    if 'base_config' in cfg:
        base=yaml.safe_load((args.config.parent/cfg['base_config']).read_text())
        cfg={**base,**cfg}
        cfg['review_cameras']={**base['review_cameras'],**cfg.get('review_cameras',{})}
        cfg['assets']={**base['assets'],**cfg.get('assets',{})}
    if cfg['layout']!='empty':
        if args.robot_config is None:
            p.error('--robot-config is required for the drone reference')
        raw=args.robot_config.read_text()
        # The locked PX4 example uses full-line // comments only. Other JSONC
        # forms fail closed in json.loads instead of silently changing strings.
        robot=json.loads('\n'.join(line for line in raw.splitlines() if not line.lstrip().startswith('//')))
        links=[]
        for link in robot['links']:
            visual=link['visual']; geo=visual['geometry']; origin=visual.get('origin',{})
            assert geo['type']=='unreal_mesh'
            rpy=[float(v) for v in origin.get('rpy-deg','0 0 0').split()]
            assert rpy==[0,0,0], 'This static reference supports the zero-rotation PX4 links only'
            assert geo.get('scale','1 1 1')=='1 1 1', 'Unexpected upstream mesh scale'
            xyz=[float(v) for v in origin.get('xyz','0 0 0').split()]
            links.append({'name':link['name'],'asset':geo['name'],'xyz_neu_m':[xyz[0],xyz[1],-xyz[2]]})
        assert len(links)==5
        cfg['drone_visual_links']=links
        cfg['drone_robot_config_name']=args.robot_config.name
        cfg['drone_robot_config_sha256']=hashlib.sha256(args.robot_config.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(cfg,indent=2)+'\n')


if __name__=='__main__':
    main()
