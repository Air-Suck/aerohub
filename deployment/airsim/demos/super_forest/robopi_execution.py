"""Candidate AS2 lifecycle; PC retains planning and mission recovery ownership."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import hashlib
import tarfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
CANDIDATE = '/data/aerohub/validation/super-follow-path/as2'


def connect():
    # Use the established local configuration without printing credentials.
    result = subprocess.check_output(['bash', '-c',
        'source "$1"; env -0', 'super-robopi', str(ROOT/'tools/robopi/load_env.sh')])
    for item in result.split(bytes([0])):
        if b'=' in item:
            key, value = item.split(b'=', 1)
            os.environ[key.decode()] = value.decode()
    sys.path.insert(0, str(ROOT/'tools/robopi'))
    import robopi
    return robopi


def dds_profile(path, local, peer):
    import ipaddress
    local, peer = str(ipaddress.IPv4Address(local)), str(ipaddress.IPv4Address(peer))
    tree = ET.parse(ROOT/'deployment/airsim/config/dds/fastdds_udp.xml')
    ns = '{http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles}'
    ET.register_namespace('', ns[1:-1])
    transport = tree.find('.//'+ns+'transport_descriptor')
    whitelist = ET.SubElement(transport, ns+'interfaceWhiteList')
    ET.SubElement(whitelist, ns+'address').text = local
    ET.SubElement(whitelist, ns+'address').text = '127.0.0.1'
    rtps = tree.find('.//'+ns+'rtps')
    peers = ET.SubElement(rtps, ns+'builtin')
    peers = ET.SubElement(peers, ns+'initialPeersList')
    for address in (peer, local, '127.0.0.1'):
        locator = ET.SubElement(peers, ns+'locator')
        udp = ET.SubElement(locator, ns+'udpv4')
        ET.SubElement(udp, ns+'address').text = address
    # Keep discovery available to participants beyond the default unicast scan range.
    locator = ET.SubElement(peers, ns+'locator')
    udp = ET.SubElement(locator, ns+'udpv4')
    ET.SubElement(udp, ns+'address').text = '239.255.0.1'
    ET.SubElement(udp, ns+'port').text = str(7400+250*94)
    tree.write(path, encoding='utf-8', xml_declaration=True)


class BoardExecution:
    def __init__(self, out):
        self.out = out
        self.link = connect()
        self.started = False
        self.env = (f'cd {shlex.quote(CANDIDATE)} && . ./board_env.sh && export '
                    'ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp '
                    'AS2_EXECUTION_BACKEND=follow_path '
                    + ' '.join(f'{key}={shlex.quote(CANDIDATE+"/"+value)}' for key,value in {
                        'FASTRTPS_DEFAULT_PROFILES_FILE':'fastdds_udp.xml',
                        'AS2_CONFIG_FILE':'config.yaml','AS2_FLEET_FILE':'fleet.yaml',
                        'AS2_CONTROLLER_CONFIG_FILE':'pid_speed_controller.yaml',
                        'AS2_RUNTIME_DIR':'runtime'}.items())+' && ')

    def check_time(self, phase):
        offset=self.link.clock_offset()
        sample={'pc_wall_seconds':time.time(),'phase':phase,
                'board_minus_pc_wall_seconds':offset,'maximum_absolute_seconds':.5}
        with (self.out/'clock-offsets.jsonl').open('a') as stream:stream.write(json.dumps(sample)+'\n')
        if phase=='startup':(self.out/'clock-offset.json').write_text(json.dumps(sample)+'\n')
        if abs(offset)>.5:
            raise RuntimeError('Board/PC wall clocks differ by more than 0.5s ('+phase+'); controlled mission recovery required')

    def prepare(self):
        self.check_time('startup')
        check_domain = """from pathlib import Path
active=[]
for p in Path('/proc').glob('[0-9]*'):
 try:
  if b'ROS_DOMAIN_ID=94' in (p/'environ').read_bytes().split(b'\\0'): active.append(p.name)
 except OSError: pass
assert not active, 'Domain 94 already has board consumers: '+str(active)
"""
        self.link.board('python3 -c '+shlex.quote(check_domain))
        manifest = json.loads(self.link.board('cat '+CANDIDATE+'/deployment.json'))
        if manifest['prefix'] != CANDIDATE or 'uav_follow_path_execution' not in manifest['packages']:
            raise RuntimeError('Candidate FollowTimedPath deployment is missing')
        check = 'import hashlib,json,pathlib; r=pathlib.Path('+repr(CANDIDATE)+'); m=json.loads((r/"deployment.json").read_text()); assert all(hashlib.sha256((r/"install"/p).read_bytes()).hexdigest()==h for p,h in m["files"].items())'
        self.link.board('python3 -c '+shlex.quote(check))
        status = json.loads(self.link.board(self.env+'python3 manage_runtime.py status || true'))
        if status.get('alive'):
            raise RuntimeError('Candidate AS2 is already running')
        if getattr(self,'require_super',False) and not {'super_planner','uav_super_adapter','as2_navigation_inputs'}.issubset(manifest['packages']):
            raise RuntimeError('All-board mode requires a candidate deployed with --include-super')
        local = self.link.required('PC_IP')
        peer = self.link.required('ROBOPI_IP')
        dds_profile(self.out/'pc-dds.xml', local, peer)
        dds_profile(self.out/'board-dds.xml', peer, local)
        for source, target in [('board-dds.xml','fastdds_udp.xml'),('fleet.yaml','fleet.yaml')]:
            self.link.hdc('file','send',str(self.out/source),CANDIDATE+'/'+target)
        (self.out/'board-deployment.json').write_text(json.dumps(manifest,indent=2)+'\n')
        (self.out/'execution.json').write_text(json.dumps({'location':'robopi','root':CANDIDATE,
            'domain':94,'pc_interface':local,'board_interface':peer,'backend':'follow_path'},indent=2)+'\n')

    def compute_command(self, action):
        remote=CANDIDATE+'/runtime/super-runs/'+self.out.name
        return self.env+'python3 super_app/manage_mission.py '+action+' --kind compute --output '+shlex.quote(remote)

    def start_compute(self):
        remote=CANDIDATE+'/runtime/super-runs/'+self.out.name
        self.link.board('mkdir -p '+shlex.quote(remote))
        state=self.link.board(self.compute_command('start'))
        (self.out/'board-compute-start.json').write_text(state+'\n')
        deadline=time.monotonic()+150
        while True:
            self.check()
            status=json.loads(self.link.board(self.compute_command('status')))
            if not status['alive']:raise RuntimeError('Compute startup exited; collect board logs')
            code="import json;from pathlib import Path;p=Path("+repr(remote+'/ready.json')+");print(p.read_text() if p.exists() else '{}')"
            ready=json.loads(self.link.board('python3 -c '+shlex.quote(code)))
            if ready.get('ready'):
                (self.out/'board-compute-ready.json').write_text(json.dumps(ready,indent=2)+'\n')
                break
            if time.monotonic()>deadline:raise RuntimeError('Compute capability did not become ready')
            time.sleep(1)
        topology=json.loads((self.out/'execution.json').read_text())
        topology.update(location='four-part',mission_location='workstation',algorithm_location='robopi',
            navigation_location='robopi',adapter_location='robopi',runtime_location='robopi',
            task_control='ROS only',simulator_rpc_location='workstation-local',remote_compute_evidence=remote)
        (self.out/'execution.json').write_text(json.dumps(topology,indent=2)+'\n')

    def stop_compute(self):
        if not (self.out/'board-compute-start.json').exists():return
        result=json.loads((self.out/'result.json').read_text()) if (self.out/'result.json').exists() else {}
        if not result.get('landed_and_disarmed') and result.get('motion_requested') is not False:
            raise RuntimeError('Compute retained until task recovery is confirmed')
        before=json.loads(self.link.board(self.compute_command('status')))
        expected=json.loads((self.out/'board-compute-start.json').read_text())
        if any(before.get(k)!=expected.get(k) for k in ('pid','ticks','boot_id')):raise RuntimeError('Compute identity changed; refusing stop')
        (self.out/'capability-after-task.json').write_text(json.dumps(before)+'\n')
        self.link.board(self.compute_command('stop'))
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            state=json.loads(self.link.board(self.compute_command('status')))
            if not state['alive']:
                (self.out/'board-compute-stop.json').write_text(json.dumps(state)+'\n')
                self.collect_mission(destination=self.out/'compute')
                return
            time.sleep(.5)
        raise RuntimeError('Compute runtime did not stop')

    def mission_command(self, action, case):
        remote=CANDIDATE+'/runtime/super-runs/'+self.out.name
        return (self.env+'python3 super_app/manage_mission.py '+action+' --output '+shlex.quote(remote)+
            ' --case '+shlex.quote(case)+' --rpc-host '+shlex.quote(self.link.required('PC_IP')))

    def collect_mission(self, destination=None):
        destination=destination or self.out
        destination.mkdir(parents=True,exist_ok=True)
        remote=CANDIDATE+'/runtime/super-runs/'+self.out.name
        archive=CANDIDATE+'/runtime/'+self.out.name+'-evidence.tar.gz'
        code="""import tarfile,hashlib
from pathlib import Path
root=Path(ROOT)
archive=Path(ARCHIVE)
with tarfile.open(archive,'w:gz') as output:
 for path in sorted(root.rglob('*')):
  if path.is_file():output.add(path,arcname=str(path.relative_to(root)))
print(hashlib.sha256(archive.read_bytes()).hexdigest())
""".replace('ROOT',repr(remote)).replace('ARCHIVE',repr(archive))
        digest=self.link.board('python3 -c '+shlex.quote(code),timeout=60)
        local=destination/'board-evidence.tar.gz'
        self.link.hdc('file','recv',archive,str(local))
        if hashlib.sha256(local.read_bytes()).hexdigest()!=digest:raise RuntimeError('Board evidence transfer hash mismatch')
        with tarfile.open(local) as bundle:
            for member in bundle.getmembers():
                path=Path(member.name)
                if path.is_absolute() or '..' in path.parts or member.issym() or member.islnk():raise RuntimeError('Unexpected board evidence member')
            bundle.extractall(destination,filter='data')
        (destination/'board-evidence-sha256.txt').write_text(digest+'\n')

    def run_mission(self, case):
        manifest=json.loads((self.out/'board-deployment.json').read_text())
        needed={'super_planner','rog_map','mars_quadrotor_msgs','uav_super_adapter','uav_planner_framework','as2_navigation_inputs'}
        if not needed.issubset(manifest['packages']):raise RuntimeError('Candidate does not contain board SUPER; deploy --include-super')
        remote=CANDIDATE+'/runtime/super-runs/'+self.out.name
        ping="import sys;sys.path.insert(0,APP);from airsim_rpc import Rpc;assert Rpc(HOST,43472,timeout=3).call('ping')".replace('APP',repr(CANDIDATE+'/super_app')).replace('HOST',repr(self.link.required('PC_IP')))
        try:self.link.board(self.env+'python3 -c '+shlex.quote(ping),timeout=15)
        except BaseException as exc:
            (self.out/'result.json').write_text(json.dumps({'success':False,'motion_requested':False,'case':case,'error':'Board AirSim RPC preflight: '+str(exc)})+'\n')
            raise
        self.link.board('mkdir -p '+shlex.quote(remote))
        self.link.hdc('file','send',str(self.out/'fleet.yaml'),remote+'/fleet.yaml')
        state=self.link.board(self.mission_command('start',case))
        (self.out/'board-mission-start.json').write_text(state+'\n')
        deadline=time.monotonic()+900
        try:
            while True:
                state=json.loads(self.link.board(self.mission_command('status',case)))
                if not state['alive']:break
                self.check()
                if time.monotonic()>deadline:raise RuntimeError('Board mission deadline exceeded')
                time.sleep(2)
        except BaseException:
            self.link.board(self.mission_command('stop',case))
            recovery_deadline=time.monotonic()+150
            while time.monotonic()<recovery_deadline:
                state=json.loads(self.link.board(self.mission_command('status',case)))
                if not state['alive']:break
                time.sleep(2)
            self.collect_mission()
            raise
        self.collect_mission()
        execution=json.loads((self.out/'execution.json').read_text())
        execution.update(location='robopi-all',algorithm_location='robopi',mission_location='robopi',
            navigation_location='robopi',adapter_location='robopi',remote_evidence=remote)
        (self.out/'execution.json').write_text(json.dumps(execution,indent=2)+'\n')
        result=json.loads((self.out/'result.json').read_text())
        if not result.get('success'):raise RuntimeError('Board SUPER mission failed; inspect result.json')

    def start(self):
        self.started = True
        state = self.link.board(self.env+'python3 manage_runtime.py start')
        (self.out/'board-start.json').write_text(state+'\n')

    def check(self):
        self.check_time('runtime')
        state=json.loads(self.link.board(self.env+'python3 manage_runtime.py status',timeout=10))
        owned=json.loads((self.out/'board-start.json').read_text())
        if not state.get('alive') or any(state.get(k)!=owned.get(k) for k in ('pid','ticks','boot_id')):
            raise RuntimeError('Candidate AS2 process lost')
        roots=[state['pid']]
        for name in ('board-mission-start.json','board-compute-start.json'):
            mission_state=self.out/name
            if mission_state.exists():roots.append(json.loads(mission_state.read_text())['pid'])
        sample = """import json,os,time
from pathlib import Path
rows={}
for p in Path('/proc').glob('[0-9]*'):
 try:
  text=(p/'stat').read_text();f=text[text.rfind(')')+2:].split()
  rows[int(p.name)]={'pid':int(p.name),'ppid':int(f[1]),'cpu_ticks':int(f[11])+int(f[12]),'rss_bytes':int(f[21])*os.sysconf('SC_PAGE_SIZE')}
 except (OSError,ValueError,IndexError):pass
owned={PID}
while True:
 new={pid for pid,r in rows.items() if r['ppid'] in owned}-owned
 if not new:break
 owned.update(new)
print(json.dumps({'monotonic':time.monotonic(),'ticks_per_second':os.sysconf('SC_CLK_TCK'),'roots':[PID],'processes':[rows[p] for p in sorted(owned) if p in rows]}))
""".replace('PID', ','.join(map(str,roots)))
        metrics=self.link.board('python3 -c '+shlex.quote(sample),timeout=10)
        with (self.out/'board-resources.jsonl').open('a') as stream:stream.write(metrics+'\n')

    def stop(self):
        if not self.started:
            return
        result_path = self.out/'result.json'
        result = json.loads(result_path.read_text()) if result_path.exists() else {}
        if not result.get('landed_and_disarmed') and result.get('motion_requested') is not False:
            raise RuntimeError('Recovery is unconfirmed; candidate AS2 retained. Confirm landing/disarm before stopping it.')
        current=json.loads(self.link.board(self.env+'python3 manage_runtime.py status || true'))
        owned=json.loads((self.out/'board-start.json').read_text())
        if any(current.get(k)!=owned.get(k) for k in ('pid','ticks','boot_id')):
            raise RuntimeError('Candidate process identity changed; refusing stop')
        state = self.link.board(self.env+'python3 manage_runtime.py stop')
        (self.out/'board-stop.json').write_text(state+'\n')
        self.started = False
