#!/usr/bin/env python3
"""One fresh YOPO 5 m/s course flight; --prepare-only does not arm."""
import argparse, datetime, fcntl, json, os, re, signal, subprocess as sp, time
from pathlib import Path
from urllib.request import build_opener, ProxyHandler
import run_super as c
ROOT=c.ROOT
APP=Path(__file__).resolve().parent
STATE=ROOT/'log/projectairsim/yopo-course-session.json'

def running(name):
    result=sp.run(['docker','inspect',name,'--format','{{.State.Running}}'],capture_output=True,text=True)
    return result.returncode==0 and result.stdout.strip()=='true'

def stop_worker(worker):
    try:
        pid=worker['pid']
        if Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]==worker['start_ticks']:
            os.killpg(pid,signal.SIGTERM)
    except (FileNotFoundError,ProcessLookupError):pass

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--speed', type=float, default=5., choices=[3.,5.])
    args=parser.parse_args()
    if running(c.PX4) and re.search(r'arming_state:\s*2\b',c.status()):
        raise RuntimeError('PX4仍解锁，先等待降落上锁。')
    active=c.capture(['docker','ps','--format','{{.Names}}']).splitlines()
    if any(n=='aerohub-wmm5-flight' or n.startswith('aerohub-manual5-flight-') for n in active):
        raise RuntimeError('SUPER任务仍在运行。')
    if STATE.exists():
        old=json.loads(STATE.read_text())
        if old.get('flight') and running(old['flight']):raise RuntimeError('上一轮YOPO尚未结束。')
        if old.get('compute') and running(old['compute']):c.run(['docker','stop','--timeout','20',old['compute']])
        if old.get('worker'):stop_worker(old['worker'])
    tag=datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    out=ROOT/'log/projectairsim'/('yopo-course-'+tag);out.mkdir()
    model_dir=ROOT/'src/inference/models/yopo'
    python=model_dir/'.venv/bin/python'
    if not python.exists():
        raise RuntimeError('先通过 deployment/inference/setup_models.py 准备 YOPO。')
    probe=sp.run([str(python),'-c',
        'import ctypes,sys; status=ctypes.CDLL("libcuda.so.1").cuInit(0); print("CUDA cuInit:",status); sys.exit(0 if status==0 else 1)'],
        capture_output=True,text=True)
    (out/'cuda-preflight.txt').write_text(probe.stdout+probe.stderr)
    if probe.returncode:
        raise RuntimeError('CUDA初始化失败，未复位场景或起飞。'+probe.stdout+probe.stderr)
    (out/'predeclared.json').write_text(json.dumps(dict(planner='yopo',runs=1,speed_m_s=args.speed,
        acceleration='native YOPO scaling',goal_relative_enu_m=[0,120,2],
        arrival='native ARRIVED_5M followed by physical stop',collision_coverage='unverified native collision topic not bridged'),indent=2))
    c.run([json.loads(c.PROFILE.read_text())['client_python'],APP/'prepare_yopo_scene.py'],cwd=ROOT)
    c.run(['docker','stop','--timeout','15',c.PX4]);c.manager('stop')
    c.PROFILE=ROOT/'log/projectairsim/yopo-course-config/profile.json'
    c.wait_ports_free();c.manager('start');c.run(['docker','start',c.PX4])
    deadline=time.monotonic()+90
    while time.monotonic()<deadline:
        try:
            status=c.status()
            if 'pre_flight_checks_pass: True' in status:
                (out/'px4-preflight.txt').write_text(status);break
        except sp.CalledProcessError:pass
        time.sleep(2)
    else:raise RuntimeError('PX4未就绪，没有起飞。')
    env=dict(os.environ,INFERENCE_PORT='18081',YOPO_VELOCITY=str(args.speed),YOPO_DEVICE='cuda',
             PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(ROOT/'src/inference/runtime'))
    with (out/'worker.log').open('w') as log:
        worker=sp.Popen(['bash',str(ROOT/'deployment/inference/start_yopo.sh')],
             cwd=ROOT,env=env,stdout=log,stderr=sp.STDOUT,start_new_session=True)
    session=dict(output=str(out),worker=dict(pid=worker.pid,start_ticks=Path(f'/proc/{worker.pid}/stat').read_text().rsplit(')',1)[1].split()[19]))
    STATE.write_text(json.dumps(session,indent=2))
    http=build_opener(ProxyHandler({}));deadline=time.monotonic()+90
    while time.monotonic()<deadline:
        if worker.poll() is not None:raise RuntimeError('worker退出，见 '+str(out/'worker.log'))
        try:
            with http.open('http://127.0.0.1:18081/health/ready',timeout=2) as response:metadata=json.load(response)
            (out/'worker-ready.json').write_text(json.dumps(metadata,indent=2));break
        except Exception:time.sleep(1)
    else:raise RuntimeError('worker未就绪。')
    base=['docker','run','-d','--init','--network','host','--ipc','host','-e','ROS_DOMAIN_ID=188',
          '-v',str(ROOT)+':/repo','-v',str(out)+':/evidence']
    setup=('source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && '
           'source /root/aerostack2_ws/install/setup.bash && source /opt/uav_execution_ws/install/setup.bash && '
           'source /repo/log/yopo-single/ros_ws/install/setup.bash && '
           'export PYTHONPATH=/repo/src/planner/yopo/ros2_adapter:/repo/src/inference/ros2_adapter:$PYTHONPATH && ')
    compute='aerohub-yopo-course-compute-'+tag;session['compute']=compute;STATE.write_text(json.dumps(session,indent=2))
    c.run(base+['--name',compute,'-e','AS2_EXECUTION_BACKEND=follow_path','-e','YOPO_COURSE_SPEED='+str(args.speed),
        '-e','AS2_FLEET_FILE=/repo/deployment/airsim/demos/projectairsim_course/fleet.yaml',
        '-e','AS2_CONTROLLER_CONFIG_FILE=/repo/deployment/airsim/demos/yopo_single/controller.yaml',
        '-e','AS2_TIMED_ACCELERATION_CONTINUITY_TOLERANCE=0.5',
        '--entrypoint','bash',c.IMAGE,'-lc',setup+'exec python3 /repo/deployment/airsim/demos/projectairsim_course/yopo_compute.py'])
    flight='aerohub-yopo-course-flight-'+tag;session['flight']=flight;STATE.write_text(json.dumps(session,indent=2))
    cmd=setup+'exec python3 /repo/deployment/airsim/demos/projectairsim_course/yopo_mission.py --output /evidence --timeout 300'
    if args.prepare_only:cmd+=' --ground-only'
    c.run(base+['--name',flight,'--entrypoint','bash',c.IMAGE,'-lc',cmd])
    print(f'YOPO {args.speed}m/s；本轮证据：'+str(out),flush=True)
    try:
        while running(flight):
            path=out/'mission.jsonl'
            if path.exists():
                with path.open('rb') as stream:
                    stream.seek(max(0,path.stat().st_size-8192));lines=stream.read().decode(errors='replace').splitlines()
                if lines:
                    try:
                        row=json.loads(lines[-1]);print(row.get('state'),row.get('pose'),row.get('reason'),flush=True)
                    except ValueError:pass
            time.sleep(3)
    except KeyboardInterrupt:
        print('请求停止并降落，等待恢复……',flush=True)
        sp.run(['docker','stop','--timeout','150',flight],check=False)
    finally:
        (out/'mission-console.log').write_text(c.capture(['docker','logs',flight]))
        final=c.status();(out/'px4-final-status.txt').write_text(final)
        if not re.search(r'arming_state:\s*2\b',final):
            c.run(['docker','stop','--timeout','20',compute])
            stop_worker(session['worker'])
            try:worker.wait(timeout=15)
            except sp.TimeoutExpired:print('worker未及时退出。')
        else:print('PX4仍解锁，保留执行链供恢复。',flush=True)
    result=out/'result.json'
    if result.exists():
        report=json.loads(result.read_text());print(json.dumps(report,indent=2))
        if not report.get('success'):raise SystemExit(1)
    else:raise RuntimeError('无result.json，未完成。')

if __name__=='__main__':
    with (ROOT/'log/projectairsim/course-flight.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        main()
