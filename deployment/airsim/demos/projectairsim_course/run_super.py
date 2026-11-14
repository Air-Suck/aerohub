#!/usr/bin/env python3
"""Local repeat entry: one WMM course flight per invocation, with fresh scene/SITL."""
import datetime
import json
import os
from pathlib import Path
import re
import shutil
import socket
import errno
import argparse
import subprocess as sp
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
PROFILE = ROOT / 'log/projectairsim/package-course-wmm/profile.json'
RUNTIME = PROFILE.parent / 'runtime'
PX4 = 'aerohub-course-wmm-px4'
IMAGE = 'uav/apps-super-forest:onboard-v0.1.1'
SOURCE = Path(__file__).resolve().parent
ENV = dict(os.environ, DISPLAY=os.environ.get('DISPLAY', ':1'))
# Respect the caller's desktop credentials; discover GDM only for the current UID.
if 'XAUTHORITY' not in ENV:
    authority = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')) / 'gdm/Xauthority'
    if authority.is_file():
        ENV['XAUTHORITY'] = str(authority)

def run(args, **kwargs):
    return sp.run([str(a) for a in args], check=True, **kwargs)

def capture(args):
    return run(args, capture_output=True, text=True).stdout

def status():
    return capture(['docker', 'exec', PX4, '/opt/px4-gazebo/bin/px4-listener',
                    '--instance', '20', 'vehicle_status', '-n', '1'])

def manager(action):
    run([sys.executable, ROOT/'simulator/manager/manager.py', action,
         '--profile', PROFILE, '--run-dir', RUNTIME], env=ENV, cwd=ROOT)

def wait_ports_free(timeout=90):
    profile = json.loads(PROFILE.read_text())
    ports = list(profile['ports'].values()) + [profile['display']['port']]
    deadline = time.monotonic() + timeout
    while True:
        sockets = []
        blocked = None
        try:
            for port in ports:
                sock = socket.socket()
                sockets.append(sock)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    sock.bind(('0.0.0.0', port))
                except OSError as exc:
                    if exc.errno != errno.EADDRINUSE:
                        raise
                    blocked = port
                    break
        finally:
            for sock in sockets:
                sock.close()
        if blocked is None:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(f'端口 {blocked} 在停止场景后仍被占用；未启动飞行。请用 ss -ltnp 检查归属。')
        print(f'等待旧场景端口 {blocked} 释放……', flush=True)
        time.sleep(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true', help='只复位场景与飞控，不起飞')
    args = parser.parse_args()
    # Check ownership and active missions before changing the existing runtime.
    active = capture(['docker', 'ps', '--format', '{{.Names}}']).splitlines()
    busy = [n for n in active if n == 'aerohub-wmm5-flight' or n.startswith('aerohub-manual5-flight-') or n.startswith('aerohub-yopo-course-')]
    if busy:
        raise RuntimeError('已有飞行任务运行，请等它结束：' + ', '.join(busy))
    if PX4 in active and re.search(r'arming_state:\s*2\b', status()):
        raise RuntimeError('PX4 仍处于解锁状态，拒绝重置；先结束当前飞行并降落上锁。')
    tag = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    out = ROOT/'log/projectairsim'/('manual-wmm5-'+tag)
    out.mkdir()
    shutil.copytree(SOURCE/'super', out/'app')
    shutil.copy2(SOURCE/'fleet.yaml', out/'fleet.yaml')
    task = json.loads((out/'app/task.json').read_text())
    assert task['speed_m_s'] == 5 and task['max_acc_m_s2'] == 2
    (out/'predeclared.json').write_text(json.dumps({
        'runs': 1, 'speed_m_s': 5, 'acceleration_m_s2': 2,
        'goal_enu_m': [0,120,2], 'arrival_tolerance_m': 1.5,
        'collision_coverage': 'unverified native collision topic not bridged',
        'package': 'BlocksPlannerCourse-linux-v7-wmm'}, indent=2))
    # Correct the local historical fallback's hardcoded system id for this vehicle.
    mission = out/'app/mission.py'
    mission.write_text(mission.read_text().replace('target_system=61,', 'target_system=21,'))
    print('本轮证据：', out, flush=True)
    run(['docker', 'stop', '--timeout', '15', PX4])
    manager('stop')
    wait_ports_free()
    manager('start')
    run(['docker', 'start', PX4])
    deadline = time.monotonic()+90
    while time.monotonic() < deadline:
        try:
            state = status()
            if 'pre_flight_checks_pass: True' in state and re.search(r'arming_state:\s*1\b', state):
                (out/'px4-preflight.txt').write_text(state)
                break
        except sp.CalledProcessError:
            pass
        time.sleep(2)
    else:
        raise RuntimeError('PX4 90秒内未就绪，本轮没有起飞。')
    if args.prepare_only:
        print('场景与PX4已就绪；prepare-only没有启动飞行。', flush=True)
        return
    base = ['docker','run','-d','--init','--network','host','--ipc','host','-e','ROS_DOMAIN_ID=188']
    bag = 'aerohub-manual5-bag-'+tag
    flight = 'aerohub-manual5-flight-'+tag
    bag_cmd = ('source /opt/ros/humble/setup.bash && source /opt/super_ws/install/setup.bash && '
               'exec ros2 bag record -o /evidence/map-bag /rog_map/occ /rog_map/inf_occ '
               '/drone0/sensor_measurements/lidar1/points /drone0/navigation/points '
               '/drone0/navigation/odom /tf /tf_static /clock /planning_cmd/poly_traj')
    run(base+['--name',bag,'-v',str(out)+':/evidence','--entrypoint','bash',IMAGE,'-lc',bag_cmd])
    started = False
    try:
        cmd = ('source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && '
               'source /root/aerostack2_ws/install/setup.bash && source /opt/uav_execution_ws/install/setup.bash && '
               'source /opt/super_ws/install/setup.bash && source /opt/super_app_ws/install/setup.bash && '
               'exec python3 /course/app/mission.py')
        run(base+['--name',flight,'-e','SUPER_APP_ROOT=/course/app','-e','SUPER_EVIDENCE_ROOT=/course',
                  '-e','SUPER_CASE=course','-e','SUPER_HUD_PORT=9011','-v',str(out)+':/course',
                  '-v',str(ROOT/'src/planner/super/ros2_adapter/bridge/uav_super_adapter/polynomial.py')+
                  ':/opt/super_app_ws/install/lib/python3.10/site-packages/uav_super_adapter/polynomial.py:ro',
                  '--entrypoint','bash',IMAGE,'-lc',cmd])
        started = True
        print('已启动。Ctrl+C 会请求结束任务并执行降落恢复，请等待收口。', flush=True)
        while capture(['docker','inspect',flight,'--format','{{.State.Running}}']).strip() == 'true':
            path = out/'flight.jsonl'
            if path.exists():
                # Tail a bounded amount of the growing evidence file.
                with path.open('rb') as stream:
                    stream.seek(max(0, path.stat().st_size-16384))
                    rows = stream.read().decode(errors='replace').splitlines()
                for row in reversed(rows):
                    try:
                        sample = json.loads(row)
                    except ValueError:
                        continue
                    if 'phase' in sample:
                        print(sample['phase'], sample.get('position'), flush=True)
                        break
            time.sleep(3)
    except KeyboardInterrupt:
        print('\n请求降落恢复；等待任务退出……', flush=True)
        if started:
            sp.run(['docker','stop','--timeout','150',flight], check=False)
    finally:
        sp.run(['docker','kill','--signal','SIGINT',bag], check=False)
        sp.run(['docker','wait',bag], check=False)
        if started:
            (out/'mission-console.log').write_text(capture(['docker','logs',flight]))
        try:
            final = status()
            (out/'px4-final-status.txt').write_text(final)
            if re.search(r'arming_state:\s*2\b', final):
                print('注意：PX4 仍解锁，恢复未完成，禁止直接开启下一轮。', flush=True)
        except sp.CalledProcessError:
            print('未取得最终PX4状态，请检查飞控。', flush=True)
    result = out/'result.json'
    if result.exists():
        print(result.read_text())
    else:
        print('任务没有生成 result.json，视为未完成。')
    print('证据目录：', out)

if __name__ == '__main__':
    import fcntl
    # Prevent simultaneous terminal invocations from racing during the reset.
    with (ROOT/'log/projectairsim/course-flight.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        main()
