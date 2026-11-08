"""Package and deploy persistent AS2/algorithm overlays; no simulation or flight."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import struct
import re

ROOT = Path(__file__).resolve().parents[2]
AS2 = '/data/aerohub/as2'
APP = '/data/aerohub/lib/catora'
COMMON_PACKAGES = {'uav_planner_interfaces', 'uav_planner_framework', 'uav_as2_setpoint_adapter'}
APP_PACKAGES = COMMON_PACKAGES | {'mrs_formation_reshaping', 'uav_cat_ora_adapter'}
APPLICATIONS = {
    'catora': APP_PACKAGES,
    'swarm-formation': COMMON_PACKAGES | {'swarm_graph', 'traj_utils', 'plan_env', 'path_searching',
                                         'traj_opt', 'ego_planner', 'uav_swarm_formation_adapter', 'as2_navigation_inputs'},
}
AS2_PACKAGES = {'uav_planner_interfaces', 'uav_follow_path_execution', 'as2_behavior', 'as2_behaviors_motion', 'as2_core', 'as2_motion_controller',
                'as2_motion_reference_handlers', 'as2_msgs', 'as2_platform_pixhawk', 'as2_state_estimator',
                'mocap4r2_msgs', 'px4_msgs', 'rmw_fastrtps_shared_cpp', 'vision_msgs'}
OLD = '/data/local/tmp/aerohub-as2'


def relocate_elf(path):
    """Remove build-machine search paths in a staged ELF without changing code or dependencies."""
    data = bytearray(path.read_bytes())
    if data[4:6] != bytes((2, 1)) or struct.unpack_from('<H', data, 18)[0] != 183:
        raise RuntimeError('Expected ELF64 little-endian AArch64: '+str(path))
    phoff = struct.unpack_from('<Q', data, 32)[0]
    entsize, count = struct.unpack_from('<HH', data, 54)
    segments = [struct.unpack_from('<IIQQQQQQ', data, phoff+i*entsize) for i in range(count)]
    dynamic = next((p for p in segments if p[0] == 2), None)
    if dynamic is None: return
    entries = []
    for offset in range(dynamic[2], dynamic[2]+dynamic[5], 16):
        tag, value = struct.unpack_from('<qQ', data, offset)
        if tag == 0: break
        entries.append((tag, value))
    strings = next((v for t,v in entries if t == 5), None)
    if strings is None: return
    load = next(p for p in segments if p[0] == 1 and p[3] <= strings < p[3]+p[5])
    base = load[2]+strings-load[3]
    for tag, value in entries:
        if tag not in (15,29): continue
        begin = base+value
        end = data.index(0, begin)
        old = bytes(data[begin:end])
        if any(prefix in old for prefix in (b'/work/', b'/target-out', OLD.encode(), b'/data/aerohub/')):
            replacement = b'$ORIGIN'
            if len(replacement) > len(old): raise RuntimeError('RPATH string too short')
            data[begin:end] = replacement+b'\0'*(end-begin-len(replacement))
    path.write_bytes(data)


def package(source, destination, component):
    packages = {p.name for p in source.iterdir() if (p/'share'/p.name/'package.xml').is_file()}
    groups = ['as2', *APPLICATIONS] if component == 'all' else [component]
    changed = []
    for group in groups:
        required = AS2_PACKAGES if group == 'as2' else APPLICATIONS[group]
        selected = sorted(packages & required)
        if not selected or (group != 'as2' and set(selected) != required):
            raise RuntimeError('Missing packages for '+group+': '+str(sorted(required-packages)))
        prefix = AS2 if group == 'as2' else '/data/aerohub/lib/'+group
        mapping = {name: AS2+'/install/'+name for name in AS2_PACKAGES}
        if group != 'as2': mapping.update({name:prefix+'/install/'+name for name in required})
        for name in selected:
            namespace = source.parent/'src'/name/name
            if (namespace.parent/'setup.py').exists():
                for module in namespace.rglob('*.py'):
                    installed = source/name/'lib/python3.12/site-packages'/module.relative_to(namespace.parent)
                    if not installed.exists() or module.read_bytes()!=installed.read_bytes():
                        raise RuntimeError('Stale Python installation; run scripts/build_lib.sh: '+str(installed))
            target = destination/group/'install'/name
            shutil.copytree(source/name, target, symlinks=False, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
            for path in target.rglob('*'):
                if not path.is_file(): continue
                data = path.read_bytes()
                if data.startswith(b'\x7fELF'):
                    before = subprocess.check_output(['readelf', '-d', str(path)], text=True, stderr=subprocess.DEVNULL)
                    relocate_elf(path)
                    after = subprocess.check_output(['readelf', '-d', str(path)], text=True, stderr=subprocess.DEVNULL)
                    needed = lambda value: [line.split('Shared library:',1)[1] for line in value.splitlines() if 'NEEDED' in line]
                    if needed(before) != needed(after): raise RuntimeError('ELF dependencies changed: '+str(path))
                    for line in after.splitlines():
                        if ('RPATH' in line or 'RUNPATH' in line) and any(v in line for v in (OLD, '/work/', '/target-out')):
                            raise RuntimeError('Unresolved runtime path: '+str(path))
                    continue
                if b'\0' in data: continue
                try: text = data.decode('utf-8')
                except UnicodeDecodeError: continue
                updated = text
                for dependency, final in mapping.items():
                    for old in (OLD+'/install/', '/work/ws/install/', AS2+'/install/',
                                *('/data/aerohub/lib/'+app+'/install/' for app in APPLICATIONS)):
                        updated = re.sub(re.escape(old+dependency)+r"(?=[/\s\"'`;:]|$)", lambda _: final, updated)
                if updated.startswith('#!') and 'python' in updated.split('\n', 1)[0]:
                    updated = '#!/sys_prod/robot/out/bin/python3\n'+updated.split('\n', 1)[1]
                if OLD+'/install/' in updated: raise RuntimeError('Unresolved old prefix: '+str(path))
                if updated != text:
                    path.write_text(updated)
                    changed.append(str(path.relative_to(destination)))
        directory = destination/group
        install = directory/'install'
        manifest = {'prefix':prefix, 'packages':selected,
                    'files':{str(p.relative_to(install)):hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in sorted(install.rglob('*')) if p.is_file()}}
        (directory/'deployment.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--component', choices=['as2', *APPLICATIONS, 'all'], default='all')
    parser.add_argument('--install', type=Path)
    parser.add_argument('--activate-config', action='store_true', help='Switch .env ROBOPI_ROOT after deployment; omit while old runtime is in use')
    parser.add_argument('--prepare-only', type=Path, help='Create a local bundle without contacting the device')
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT/'scripts'))
    from robopi import board, hdc
    work = Path(os.environ.get('OHOS_WORK_ROOT', ROOT/'artifacts/ohos-as2'))
    if not work.is_absolute(): work = ROOT/work
    source = args.install or work/'ws/install'
    with tempfile.TemporaryDirectory(prefix='aerohub-deploy-') as temporary:
        stage = Path(temporary)
        package(source, stage, args.component)
        if (stage/'as2').exists():
            files = {'board_env.sh': ROOT/'deployment/robopi/board_env.sh',
                     'board_smoke.py': ROOT/'deployment/robopi/diagnostics/board_smoke.py',
                     'manage_runtime.py': ROOT/'deployment/robopi/manage_runtime.py',
                     'run_as2.py': ROOT/'runtime/as2/run.py',
                     'selected_behaviors.launch.py': ROOT/'runtime/as2/selected_behaviors.launch.py',
                     'scoped_tf.launch.py': ROOT/'runtime/as2/scoped_tf.launch.py',
                     'verify_interaction.py': ROOT/'runtime/as2/scripts/verify_interaction.py',
                     'verify_sensors.py': ROOT/'runtime/airsim_sensors/scripts/verify_ros2.py',
                     'config.yaml': ROOT/'config/as2/config.yaml', 'fleet.yaml': ROOT/'config/as2/fleet.yaml',
                     'pid_speed_controller.yaml': ROOT/'config/as2/pid_speed_controller.yaml',
                     'settings.json': ROOT/'config/airsim/settings/px4-sensors.json'}
            for name, path in files.items(): shutil.copyfile(path, stage/'as2'/name)
            if os.environ.get('ROBOPI_IP'):
                from robopi import profile
                profile(stage/'as2/fastdds_udp.xml', os.environ['ROBOPI_IP'])
            else:
                shutil.copyfile(ROOT/'config/dds/fastdds_udp.xml', stage/'as2/fastdds_udp.xml')
            if (work/'current-sources.json').exists():
                shutil.copyfile(work/'current-sources.json', stage/'as2/source-manifest.json')
        for app in APPLICATIONS:
            if (stage/app).exists():
                text = (ROOT/'deployment/robopi/catora_env.sh').read_text().replace('/lib/catora/', '/lib/'+app+'/')
                (stage/app/'env.sh').write_text(text)
        bundle = args.prepare_only or ROOT/'artifacts/robopi-runtime/persistent-overlay.tar.gz'
        bundle.parent.mkdir(parents=True, exist_ok=True)
        with tarfile.open(bundle, 'w:gz', format=tarfile.GNU_FORMAT) as archive:
            for child in stage.iterdir(): archive.add(child, arcname=child.name)
    if args.prepare_only:
        print(bundle)
        return
    # The shell entrypoint holds the shared operation lock throughout deployment.
    check = """from pathlib import Path
roots=('/data/aerohub/as2/','/data/aerohub/lib/')
active=[]
for p in Path('/proc').glob('[0-9]*'):
 try:
  argv=(p/'cmdline').read_bytes().decode(errors='replace').split(chr(0))
  env=(p/'environ').read_bytes().decode(errors='replace').split(chr(0))
  if any(a.startswith(roots) for a in argv) or any(e.startswith(('AMENT_PREFIX_PATH=','LD_LIBRARY_PATH=','CATORA_APP_ROOT=')) and any(r in e for r in roots) for e in env): active.append(p.name)
 except OSError: pass
assert not active, 'Deployment consumers are running: '+str(active)
"""
    board('python3 -c '+shlex.quote(check))
    if args.component in APPLICATIONS:
        board('test -f /data/aerohub/as2/board_env.sh')
    board('mkdir -p /data/aerohub/lib /data/aerohub/.deploy')
    remote = '/data/aerohub/.deploy/overlay.tar.gz'
    hdc('file', 'send', str(bundle), remote)
    digest = hashlib.sha256(bundle.read_bytes()).hexdigest()
    verify = f"import hashlib; assert hashlib.sha256(open({remote!r},'rb').read()).hexdigest()=={digest!r}"
    board('python3 -c '+shlex.quote(verify))
    # Board-side extraction and replacement is shared by AS2 and application deployment.
    hdc('file', 'send', str(ROOT/'deployment/robopi/install_overlay.py'), '/data/aerohub/.deploy/install_overlay.py')
    board('python3 /data/aerohub/.deploy/install_overlay.py')
    if args.activate_config and args.component in ('all', 'as2'):
        env_file = ROOT/'.env'
        if env_file.exists():
            lines = env_file.read_text().splitlines()
            lines = ['ROBOPI_ROOT='+AS2 if line.startswith('ROBOPI_ROOT=') else line for line in lines]
            env_file.write_text('\n'.join(lines)+'\n')
    print('Persistent deployment complete: '+args.component)


if __name__ == '__main__':
    main()
