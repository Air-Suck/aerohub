#!/usr/bin/env python3
"""Validate the actual packaged runtime via native RPC and HUD HTTP, without flight."""
import argparse
import json
import math
import time
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from projectairsim import ProjectAirSimClient

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--topic-port',type=int,default=9989)
p.add_argument('--service-port',type=int,default=9990)
p.add_argument('--display-port',type=int,default=8993)
a=p.parse_args()

def http(path,method='GET',payload=None):
    request=Request(f'http://127.0.0.1:{a.display_port}/aerohub/'+path,
        data=json.dumps(payload).encode() if payload is not None else (b'' if method=='POST' else None),
        method=method,headers={'Content-Type':'application/json'})
    with urlopen(request,timeout=5) as response:return json.load(response)

c=ProjectAirSimClient('127.0.0.1',a.topic_port,a.service_port)
c.connect_services()
try:
    def rpc(method,params=None):
        return c.request({'method':'/Sim/ScenePlannerCourse/'+method,'params':params or {},'version':1.0})
    names=rpc('ListObjects',{'name':'.*'})
    poses=rpc('GetObjectPoses',{'object_names':names})
    pillars=[];unexpected_references=[]
    for name,pose in zip(names,poses):
        xyz=pose['translation'];x,y,z=xyz['x'],xyz['y'],xyz['z']
        if 155<x<270 and abs(y)<6 and -4.2<z<-3.8:
            pillars.append({'name':name,'pose':pose})
        if name!='Drone1' and abs(x-151.5)<0.7 and abs(y)<0.7 and -1.6<z<-1.12:
            unexpected_references.append(name)
    assert len(pillars)==24,len(pillars)
    assert not unexpected_references,unexpected_references
    assert names.count('Drone1')==1
    state=rpc('robots/Drone1/GetGroundTruthKinematics')
    pos=state['pose']['position'];assert abs(pos['x']-151.5)<0.05 and abs(pos['y'])<0.05
    speed=math.sqrt(sum(state['twist']['linear'][axis]**2 for axis in ['x','y','z']))
    assert speed<0.1,speed
    http('telemetry','POST',{'vehicle':'Drone1','speed_mps':speed})
    status=http('status');assert status['vehicle_present'] and status['observed_speed_mps'] is not None
    http('mission/reset','POST')
    try:
        http('mission/finish','POST')
        raise AssertionError('Finish from idle must fail')
    except HTTPError as exc:assert exc.code==400
    http('mission/start','POST');time.sleep(0.35)
    running=http('status');assert running['mission']=='running' and running['elapsed_wall_seconds']>=0.3
    finished=http('mission/finish','POST');time.sleep(0.25)
    frozen=http('status');assert frozen['elapsed_wall_seconds']==finished['elapsed_wall_seconds']
    http('mission/reset','POST')
    a.output.mkdir(parents=True,exist_ok=True)
    report={'packaged_runtime':True,'scene':'ScenePlannerCourse','pillar_count':len(pillars),
        'static_reference_absent':True,'robot_count':1,'robot_position_native_ned_m':pos,
        'stationary_ground_truth_speed_mps':speed,'hud_real_telemetry':status,
        'hud_timer_running':running,'hud_timer_frozen':frozen,'invalid_finish_http':400,
        'controller_test':'SimpleFlight static scene; PX4/ROS/planner not started','flight_started':False,
        'pillars':pillars}
    (a.output/'runtime-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print('PASS: packaged 24 pillars, one real start robot, no static reference, native telemetry and HUD lifecycle; no flight.')
finally:c.disconnect()
