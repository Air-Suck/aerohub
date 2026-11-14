#!/usr/bin/env python3
"""Build the reviewed course and HUD as an independent Linux Development package."""
import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ue-root',type=Path,required=True)
    p.add_argument('--project',type=Path,required=True)
    p.add_argument('--source',type=Path,required=True,help='ProjectAirSim source tree')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--magnetic-reference',type=Path,help='Explicit local world NED field and provenance JSON')
    p.add_argument('--finalize-only',action='store_true',help='Finish bundle metadata after successful UAT archive')
    args=p.parse_args()
    magnetic_reference=None
    if args.magnetic_reference:
        import commentjson  # fail before UAT if the matching client parser is missing
        magnetic_reference=json.loads(args.magnetic_reference.read_text())
        field=magnetic_reference['field_ned_gauss']
        if len(field)!=3 or not all(math.isfinite(v) for v in field) or sum(v*v for v in field)<=0:
            raise ValueError('field_ned_gauss requires three finite, nonzero vector components')
    repo=Path(__file__).resolve().parents[3]
    project=args.project.resolve(); output=args.output.resolve(); evidence=args.evidence.resolve()
    evidence.mkdir(parents=True,exist_ok=True)
    runtime=json.loads((evidence/'runtime-map.json').read_text())
    assert runtime['runtime_map']=='/Game/BlocksPlannerCourse'
    if not args.finalize_only and shutil.disk_usage(project.parent).free<8*1024**3:
        raise RuntimeError('At least 8 GiB free space required for build and staging')
    plugin=repo/'simulator/projectairsim/unreal/AerohubDisplay'
    installed=project.parent/'Plugins/AerohubDisplay'
    for directory in ['Source','Resources']:
        for source_file in (plugin/directory).rglob('*'):
            if source_file.is_file():
                target=installed/source_file.relative_to(plugin)
                if not target.exists() or target.read_bytes()!=source_file.read_bytes():
                    raise RuntimeError('Installed HUD differs from repository: '+str(target))
    desc=json.loads(project.read_text())
    backup=evidence/'project-before-packaging.json'
    if not backup.exists():
        backup.write_text(json.dumps(desc,indent=2)+'\n')
    entry=next((e for e in desc['Plugins'] if e['Name']=='AerohubDisplay'),None)
    if entry is None:
        desc['Plugins'].append({'Name':'AerohubDisplay','Enabled':True})
    else:
        entry['Enabled']=True
    project.write_text(json.dumps(desc,indent=2)+'\n')
    cmd=[str(args.ue_root.resolve()/'Engine/Build/BatchFiles/RunUAT.sh'),'BuildCookRun',
         '-project='+str(project),'-noP4','-nocompile','-platform=Linux','-clientconfig=Development',
         '-build','-cook','-map=/Game/BlocksPlannerCourse','-stage','-pak','-compressed',
         '-archive','-archivedirectory='+str(output),'-nodebuginfo','-utf8output',
         '-ubtargs=-MaxParallelActions=2','-AdditionalCookerOptions=-CookProcessCount=1']
    (evidence/'package-command.json').write_text(json.dumps(cmd,indent=2)+'\n')
    if args.finalize_only:
        if not (evidence/'package.log').exists() or 'BUILD SUCCESSFUL' not in (evidence/'package.log').read_text():
            raise RuntimeError('finalize-only requires a successful existing UAT archive')
    else:
        print('Building and cooking BlocksPlannerCourse; details: '+str(evidence/'package.log'),flush=True)
        if (evidence/'package.log').exists():
            n=1
            while (evidence/f'package-attempt-{n}.log').exists():
                n+=1
            (evidence/'package.log').rename(evidence/f'package-attempt-{n}.log')
        with (evidence/'package.log').open('w') as log:
            # UAT launches `env chmod`; use system utilities rather than local shims.
            subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True,
                           env={**os.environ,'PATH':'/usr/local/bin:/usr/bin:/bin'})
    binary=output/'Linux/Blocks/Binaries/Linux/Blocks'
    assert binary.is_file()
    (output/'config').mkdir(exist_ok=True)
    display={'enabled':True,'vehicle':'Drone1','camera_enabled':False,'camera':'DownCamera',
             'camera_width':320,'speed_max_mps':20,'port':8991}
    (output/'config/display.json').write_text(json.dumps(display,indent=2)+'\n')
    (output/'sim_config').mkdir(exist_ok=True)
    upstream=args.source/'client/python/example_user_scripts/sim_config'
    for suffix,robot in [('px4','robot_quadrotor_px4_sitl.jsonc'),('simpleflight','robot_quadrotor_fastphysics.jsonc')]:
        if magnetic_reference:
            robot_config=commentjson.loads((upstream/robot).read_text())
            magnetometers=[sensor for sensor in robot_config['sensors'] if sensor['type']=='magnetometer']
            if not magnetometers:
                raise ValueError('Robot has no native magnetometer')
            for sensor in magnetometers:
                sensor['magnetic-field-ned-gauss']=' '.join(format(v,'.10g') for v in magnetic_reference['field_ned_gauss'])
            (output/'sim_config'/robot).write_text(json.dumps(robot_config,indent=2)+'\n')
        else:
            shutil.copy2(upstream/robot,output/'sim_config'/robot)
        scene={'id':'ScenePlannerCourse','actors':[{'type':'robot','name':'Drone1',
            'origin':{'xyz':' '.join(str(v) for v in runtime['robot_spawn_native_ned_m']),'rpy-deg':'0 0 0'},
            'robot-config':robot}],
            'clock':{'type':'steppable','step-ns':3000000,'real-time-update-rate':3000000,'pause-on-start':False},
            'home-geo-point':{'latitude':47.641468,'longitude':-122.140165,'altitude':122},
            'segmentation':{'initialize-ids':True,'ignore-existing':False,'use-owner-name':True},'scene-type':'UnrealNative'}
        (output/'sim_config'/('scene_course_'+suffix+'.jsonc')).write_text(json.dumps(scene,indent=2)+'\n')
    (output/'run-course.sh').write_text('''#!/usr/bin/env bash
set -euo pipefail
bundle_dir=$(cd -- "$(dirname -- "$0")" && pwd)
export AEROHUB_DISPLAY_CONFIG="${AEROHUB_DISPLAY_CONFIG:-$bundle_dir/config/display.json}"
exec "$bundle_dir/Linux/Blocks/Binaries/Linux/Blocks" /Game/BlocksPlannerCourse -windowed -ResX=1280 -ResY=720 -nosound -noraytracing "$@"
''')
    (output/'run-course.sh').chmod(0o755)
    shutil.copy2(Path(__file__).with_name('load_course.py'),output/'load_scene.py')
    shutil.copy2(repo/'simulator/projectairsim/display_telemetry.py',output/'display_telemetry.py')
    shutil.copy2(repo/'config/simulator/blocks_course_two_stage.yaml',output/'config/course.yaml')
    shutil.copy2(repo/'config/simulator/blocks_course.yaml',output/'config/blocks_course.yaml')
    shutil.copy2(evidence/'runtime-map.json',output/'config/runtime-map.json')
    shutil.copy2(Path(__file__).with_name('PACKAGE_README.md'),output/'README.md')
    manifest={'runtime_map':runtime['runtime_map'],'review_map':runtime['review_map'],
              'review_map_sha256':runtime['review_map_sha256'],'configuration':'Development','platform':'Linux',
              'course_version':7,'sections_m':[60,60],'second_section':'lateral +/-15 degrees',
              'robot_spawn_native_ned_m':runtime['robot_spawn_native_ned_m'],
              'hud_sources_sha256':{str(f.relative_to(plugin)):hashlib.sha256(f.read_bytes()).hexdigest()
                  for d in ['Source','Resources'] for f in (plugin/d).rglob('*') if f.is_file()},
              'runtime_validation':'pending'}
    if magnetic_reference:
        manifest['magnetic_reference']=magnetic_reference
        (output/'config/magnetic-reference.json').write_text(json.dumps(magnetic_reference,indent=2)+'\n')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Packaged course: '+str(output),flush=True)


if __name__=='__main__':
    main()
