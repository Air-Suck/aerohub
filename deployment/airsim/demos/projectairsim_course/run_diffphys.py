#!/usr/bin/env python3
"""Fresh DiffPhys course deployment, target 5m/s, existing native execution chain."""
import argparse,datetime,fcntl,json,math,os,re,shutil,subprocess as sp,time
from pathlib import Path
import run_super as c
ROOT=c.ROOT;APP=Path(__file__).resolve().parent
IMAGE='uav/apps-super-forest:repo-native-20261109'

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
    active=c.capture(['docker','ps','--format','{{.Names}}']).splitlines()
    if any('flight-' in n or n.startswith('aerohub-diffphys-course-') for n in active):raise RuntimeError('已有任务运行，拒绝切换')
    if c.PX4 in active and re.search(r'arming_state:\s*2\b',c.status()):raise RuntimeError('PX4仍解锁')
    tag=datetime.datetime.now().strftime('%Y%m%d-%H%M%S');out=ROOT/'log/projectairsim'/('diffphys-course-'+tag);out.mkdir()
    print('DiffPhys目标5m/s，本轮证据：'+str(out),flush=True)
    shutil.copyfile(APP/'diffphys-dds.xml',out/'diffphys-dds.xml')
    import yaml
    c.run([json.loads(c.PROFILE.read_text())['client_python'],APP/'prepare_diffphys_scene.py'],cwd=ROOT)
    c.PROFILE=ROOT/'log/projectairsim/diffphys-course-config/profile.json'
    (out/'diffphys-task.json').write_text(json.dumps(dict(target_speed_m_s=5,takeoff_height_m=3,policy_profile='high_speed',goal_lookahead_m=10,command_tilt_rad=.5),indent=2))
    (out/'predeclared.json').write_text(json.dumps(dict(target_speed_m_s=5,runs=1,goal_relative_enu_m=[0,120,3],collision_coverage='unverified',arrival_tolerance_m=1.5)))
    config=yaml.safe_load((ROOT/'deployment/airsim/config/as2/config.yaml').read_text());(out/'config.yaml').write_text(yaml.safe_dump(config))
    (out/'platform.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':dict(use_sim_time=True,cmd_freq=100.,info_freq=20.,external_odom=False,fmu_prefix='/drone0',target_system_id=21)}}))
    c.run(['docker','stop','--timeout','15',c.PX4]);c.manager('stop');c.wait_ports_free();c.manager('start');c.run(['docker','start',c.PX4])
    deadline=time.monotonic()+90
    while time.monotonic()<deadline:
        try:
            status=c.status()
            if 'pre_flight_checks_pass: True' in status:break
        except sp.CalledProcessError:pass
        time.sleep(2)
    else:raise RuntimeError('PX4未就绪')
    (out/'px4-preflight.txt').write_text(status)
    import signal
    from urllib.request import build_opener,ProxyHandler
    env=dict(os.environ,DIFFPHYS_PROFILE='high_speed',DIFFPHYS_DEVICE='cuda',INFERENCE_PORT='18082',
             PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(ROOT/'src/basic_function/inference/runtime'))
    with (out/'worker.log').open('w') as stream:
        worker=sp.Popen(['bash',str(ROOT/'deployment/inference/start_diffphys.sh')],
            cwd=ROOT/'src/basic_function/inference/models/diffphys',env=env,stdout=stream,stderr=sp.STDOUT,start_new_session=True)
    http=build_opener(ProxyHandler({}));deadline=time.monotonic()+90
    while time.monotonic()<deadline:
        if worker.poll() is not None:raise RuntimeError('DiffPhys worker exited; see worker.log')
        try:
            with http.open('http://127.0.0.1:18082/health/ready',timeout=2) as response:metadata=json.load(response)
            (out/'worker-ready.json').write_text(json.dumps(metadata,indent=2));break
        except Exception:time.sleep(1)
    else:raise RuntimeError('DiffPhys worker timeout')
    base=['docker','run','-d','--init','--network','host','--ipc','host','-v',str(APP/'diffphys-dds.xml')+':/dds-profile.xml:ro','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/dds-profile.xml','-e','ROS_DOMAIN_ID=188','-e','DIFFPHYS_TEST=1','-e','AS2_BEHAVIORS=takeoff,land,go_to','-e','EXTERNAL_ATTITUDE_TEST=1','-e','DIFFPHYS_PROFILE=high_speed','-e','DIFFPHYS_SPEED=5.0','-e','DIFFPHYS_CAMERA_X=0.5','-e','DIFFPHYS_GOAL_LOOKAHEAD=10.0','-e','DIFFPHYS_COMMAND_TILT=0.5','-e','DIFFPHYS_ENDPOINT=http://127.0.0.1:18082','-e','OPENBLAS_NUM_THREADS=1','-e','OMP_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-v',str(ROOT)+':/repo:ro','-v',str(ROOT/'log/diffphys/attitude-overlay')+':/overlay:ro','-v',str(out)+':/evidence']
    setup='source /root/aerostack2_ws/install/setup.bash && source /overlay/install/setup.bash && '
    compute='aerohub-diffphys-course-compute-'+tag;flight='aerohub-diffphys-course-flight-'+tag
    extra=[]
    if args.prepare_only:(out/'hover.json').write_text(json.dumps(dict(normalized_thrust=.59,source='ground readiness placeholder; never used for flight')))
    c.run(base+extra+['--name',compute,'--entrypoint','bash',IMAGE,'-lc',setup+'exec python3 /repo/deployment/airsim/demos/projectairsim_course/diffphys_compute.py'])
    mission='/repo/deployment/airsim/demos/projectairsim_course/diffphys_ground.py' if args.prepare_only else '/repo/deployment/airsim/demos/projectairsim_course/diffphys_mission.py'
    c.run(base+['--name',flight,'--entrypoint','bash',IMAGE,'-lc',setup+'exec python3 '+mission])
    (ROOT/'log/projectairsim/diffphys-course-current.json').write_text(json.dumps(dict(output=str(out),compute=compute,flight=flight)))
    try:
        while c.capture(['docker','inspect',flight,'--format','{{.State.Running}}']).strip()=='true':
            if (out/'need-hover').exists() and not (out/'hover.json').exists():
                sample=c.capture(['docker','exec',c.PX4,'/opt/px4-gazebo/bin/px4-listener','--instance','20','vehicle_attitude_setpoint','-n','1'])
                (out/'hover-uorb.txt').write_text(sample);match=re.search(r'thrust_body:\s*\[([^\]]+)\]',sample)
                if not match:raise RuntimeError('hover sample missing')
                thrust=-float(match.group(1).split(',')[2])
                if not .1<thrust<.85:raise RuntimeError('invalid measured hover thrust')
                (out/'hover.json').write_text(json.dumps(dict(normalized_thrust=thrust,source='actual PX4 hover before DiffPhys takeover')))
            time.sleep(1)
    except KeyboardInterrupt:c.run(['docker','stop','--timeout','150',flight])
    finally:
        (out/'task-console.log').write_text(c.capture(['docker','logs',flight]));status=c.status();(out/'px4-final-status.txt').write_text(status)
        if not re.search(r'arming_state:\s*2\b',status):
            c.run(['docker','stop','--timeout','20',compute])
            if worker.poll() is None:os.killpg(worker.pid,signal.SIGTERM)
            worker.wait(timeout=15)
        else:print('飞控仍解锁，执行链保留供恢复',flush=True)
    result=out/'result.json'
    if not result.exists():raise RuntimeError('DiffPhys任务未生成result.json')
    report=json.loads(result.read_text());print(json.dumps(report,indent=2))
    if not args.prepare_only and not report.get('success'):raise SystemExit(1)

if __name__=='__main__':
    with (ROOT/'log/projectairsim/course-flight.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);main()
