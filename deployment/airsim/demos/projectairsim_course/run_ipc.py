#!/usr/bin/env python3
"""Fresh IPC course deployment, nominal 3m/s, existing native execution chain."""
import argparse,datetime,fcntl,json,math,os,re,subprocess as sp,time
from pathlib import Path
import run_super as c
ROOT=c.ROOT;APP=Path(__file__).resolve().parent
IMAGE='uav/apps-super-forest:repo-native-20261109'

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
    active=c.capture(['docker','ps','--format','{{.Names}}']).splitlines()
    if any('flight-' in n or n.startswith('aerohub-ipc-course-') for n in active):raise RuntimeError('已有任务运行，拒绝切换')
    if c.PX4 in active and re.search(r'arming_state:\s*2\b',c.status()):raise RuntimeError('PX4仍解锁')
    tag=datetime.datetime.now().strftime('%Y%m%d-%H%M%S');out=ROOT/'log/projectairsim'/('ipc-course-'+tag);out.mkdir()
    print('IPC名义3m/s，本轮证据：'+str(out),flush=True)
    import yaml
    task=json.loads((ROOT/'deployment/airsim/demos/ipc_navigation/super-3mps-balanced-task.json').read_text())
    task.update(name='ProjectAirSim course IPC nominal3',target_from_ground_enu_m=[0.,120.,2.],bounds_enu_min_m=[-6,-2,-1000],bounds_enu_max_m=[6,123,1000],map_z_min_m=-2.,map_z_max_m=12.,require_detour=False)
    (out/'forest-task.json').write_text(json.dumps(task,indent=2))
    (out/'predeclared.json').write_text(json.dumps(dict(nominal_speed_m_s=3,reference_speed_m_s=3.6,mpc_axis_limit_m_s=4,runs=1,goal_relative_enu_m=[0,120,2],collision_coverage='unverified',arrival_tolerance_m=1.5)))
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
    base=['docker','run','-d','--init','--network','host','--ipc','host','-e','ROS_DOMAIN_ID=188','-e','IPC_NATIVE=1','-e','AS2_BEHAVIORS=takeoff,land','-e','OPENBLAS_NUM_THREADS=1','-e','OMP_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-v',str(ROOT)+':/repo:ro','-v',str(ROOT/'log/ipc/control-overlay')+':/overlay:ro','-v',str(ROOT/'log/ipc/native-deps')+':/native:ro','-v',str(out)+':/evidence']
    setup='source /root/aerostack2_ws/install/setup.bash && source /overlay/install/setup.bash && source /native/ros-install/local_setup.bash && export LD_LIBRARY_PATH=/native/install/lib:$LD_LIBRARY_PATH && '
    compute='aerohub-ipc-course-compute-'+tag;flight='aerohub-ipc-course-flight-'+tag
    extra=['-e','IPC_GROUND_ONLY=1'] if args.prepare_only else []
    c.run(base+extra+['--name',compute,'--entrypoint','bash',IMAGE,'-lc',setup+'exec python3 /repo/deployment/airsim/demos/projectairsim_course/ipc_compute.py'])
    mission='/repo/deployment/airsim/demos/ipc_navigation/ground_probe.py' if args.prepare_only else '/repo/deployment/airsim/demos/projectairsim_course/ipc_mission.py'
    c.run(base+['--name',flight,'--entrypoint','bash',IMAGE,'-lc',setup+'exec python3 '+mission])
    (ROOT/'log/projectairsim/ipc-course-current.json').write_text(json.dumps(dict(output=str(out),compute=compute,flight=flight)))
    try:
        while c.capture(['docker','inspect',flight,'--format','{{.State.Running}}']).strip()=='true':
            if (out/'need-hover').exists() and not (out/'hover.json').exists():
                sample=c.capture(['docker','exec',c.PX4,'/opt/px4-gazebo/bin/px4-listener','--instance','20','vehicle_attitude_setpoint','-n','1'])
                (out/'hover-uorb.txt').write_text(sample);match=re.search(r'thrust_body:\s*\[([^\]]+)\]',sample)
                if not match:raise RuntimeError('hover sample missing')
                thrust=-float(match.group(1).split(',')[2])
                if not .1<thrust<.85:raise RuntimeError('invalid measured hover thrust')
                (out/'hover.json').write_text(json.dumps(dict(normalized_thrust=thrust,source='actual PX4 hover before IPC takeover')))
            time.sleep(1)
    except KeyboardInterrupt:c.run(['docker','stop','--timeout','150',flight])
    finally:
        (out/'task-console.log').write_text(c.capture(['docker','logs',flight]));status=c.status();(out/'px4-final-status.txt').write_text(status)
        if not re.search(r'arming_state:\s*2\b',status):c.run(['docker','stop','--timeout','20',compute])
        else:print('飞控仍解锁，执行链保留供恢复',flush=True)
    result=out/'result.json'
    if not result.exists():raise RuntimeError('IPC任务未生成result.json')
    report=json.loads(result.read_text());print(json.dumps(report,indent=2))
    if not args.prepare_only and not report.get('success'):raise SystemExit(1)

if __name__=='__main__':
    with (ROOT/'log/projectairsim/course-flight.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);main()
