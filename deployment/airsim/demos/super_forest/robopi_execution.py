"""Candidate AS2 lifecycle; PC retains planning and mission recovery ownership."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
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

    def prepare(self):
        offset=self.link.clock_offset()
        (self.out/'clock-offset.json').write_text(json.dumps({'board_minus_pc_wall_seconds':offset,'maximum_absolute_seconds':.5})+'\n')
        if abs(offset)>.5:
            raise RuntimeError('Board/PC wall clocks differ by more than 0.5s; use the existing explicit sync-time procedure before flight')
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
        local = self.link.required('PC_IP')
        peer = self.link.required('ROBOPI_IP')
        dds_profile(self.out/'pc-dds.xml', local, peer)
        dds_profile(self.out/'board-dds.xml', peer, local)
        for source, target in [('board-dds.xml','fastdds_udp.xml'),('fleet.yaml','fleet.yaml')]:
            self.link.hdc('file','send',str(self.out/source),CANDIDATE+'/'+target)
        (self.out/'board-deployment.json').write_text(json.dumps(manifest,indent=2)+'\n')
        (self.out/'execution.json').write_text(json.dumps({'location':'robopi','root':CANDIDATE,
            'domain':94,'pc_interface':local,'board_interface':peer,'backend':'follow_path'},indent=2)+'\n')

    def start(self):
        self.started = True
        state = self.link.board(self.env+'python3 manage_runtime.py start')
        (self.out/'board-start.json').write_text(state+'\n')

    def check(self):
        state=json.loads(self.link.board(self.env+'python3 manage_runtime.py status',timeout=10))
        owned=json.loads((self.out/'board-start.json').read_text())
        if not state.get('alive') or any(state.get(k)!=owned.get(k) for k in ('pid','ticks','boot_id')):
            raise RuntimeError('Candidate AS2 process lost')
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
print(json.dumps({'monotonic':time.monotonic(),'ticks_per_second':os.sysconf('SC_CLK_TCK'),'processes':[rows[p] for p in sorted(owned) if p in rows]}))
""".replace('PID', str(state['pid']))
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
