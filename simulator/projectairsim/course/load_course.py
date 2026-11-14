#!/usr/bin/env python3
"""Load the bundled native robot scene; never arm, take off or issue motion commands."""
import argparse
import json
from pathlib import Path
from projectairsim import ProjectAirSimClient, World

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--scene',choices=['px4','simpleflight'],default='px4')
p.add_argument('--address',default='127.0.0.1')
p.add_argument('--topic-port',type=int,default=8989)
p.add_argument('--service-port',type=int,default=8990)
a=p.parse_args()
root=Path(__file__).resolve().parent
client=ProjectAirSimClient(a.address,a.topic_port,a.service_port)
client.connect()
try:
    world=World(client,'scene_course_'+a.scene+'.jsonc',sim_config_path=str(root/'sim_config'),delay_after_load_sec=1)
    print(json.dumps({'scene':'ScenePlannerCourse','vehicle':'Drone1','mode':a.scene,'flight_started':False}))
finally:
    client.disconnect()
