import json,pathlib,sys,copy,math,hashlib
root=pathlib.Path.cwd();sys.path.insert(0,str(root/'simulator/projectairsim'))
import commentjson
base=root/'log/projectairsim/package-course-wmm'
profile=json.loads((base/'profile.json').read_text())
source=pathlib.Path(profile['scene_config']);dest=root/'log/projectairsim/yopo-course-config'
dest.mkdir(parents=True,exist_ok=True)
scene=commentjson.loads(source.read_text());robot=commentjson.loads((source.parent/scene['actors'][0]['robot-config']).read_text())
# Use the native sensor camera, not the stabilized Chase observer camera.
upstream_root=pathlib.Path(profile['client_path']).resolve().parents[3]
template=upstream_root/'client/python/airsimv1_scripts_migrated/multirotor/sim_config/robot_quadrotor_classic.jsonc'
classic=commentjson.loads(template.read_text())
front=copy.deepcopy(next(s for s in classic['sensors'] if s['id']=='front_center'))
(dest/'camera-template.json').write_text(json.dumps({'template':'client/python/airsimv1_scripts_migrated/multirotor/sim_config/robot_quadrotor_classic.jsonc','sensor':'front_center','sha256':hashlib.sha256(template.read_bytes()).hexdigest(),'overrides':{'id':'yopo_front','width':320,'height':185,'fov_degrees':90,'capture':'float DepthPlanar only'}},indent=2))
front['gimbal']={'lock-roll':False,'lock-pitch':False,'lock-yaw':False}
front['id']='yopo_front'  # Retain the upstream front_center pose.
# The standard sensor contract uses degrees; upstream stores this pose in radians.
if 'rpy' in front['origin']:
 front['origin']['rpy-deg']=' '.join(str(math.degrees(float(v))) for v in front['origin'].pop('rpy').split())
for c in front['capture-settings']:
 c['capture-enabled']=c['image-type']==1
 c['streaming-enabled']=False
 c['pixels-as-float']=c['image-type']==1
 c['compress']=False
 c.update(width=320,height=185,**{'fov-degrees':90})
if not any(c['image-type']==1 for c in front['capture-settings']):
 c=copy.deepcopy(front['capture-settings'][0]);c.update(**{'image-type':1,'capture-enabled':True});front['capture-settings'].append(c)
for s in robot['sensors']:
 if s['type']=='camera':
  s['enabled']=s['id']=='Chase'
  for capture in s['capture-settings']:
   capture['capture-enabled']=False
   capture['streaming-enabled']=s['id']=='Chase' and capture['image-type']==0
robot['sensors'].append(front)
scene['actors'][0]['robot-config']='robot_yopo.jsonc'
(dest/'robot_yopo.jsonc').write_text(json.dumps(robot,indent=2))
(dest/'scene_yopo.jsonc').write_text(json.dumps(scene,indent=2))
profile['scene_config']=str(dest/'scene_yopo.jsonc');profile['sensor_options']={'Drone1':{'yopo_front':{'enabled':True,'position':[0.46,0,0],'rotation':[0,0,0],'width':320,'height':185,'fov':90,'captures':['depth_planar']},'IMU1':{'enabled':True},'lidar1':{'type':'lidar','enabled':True,'position':[0,0,.2],'channels':16,'range':100,'points_per_second':100000,'rotation_hz':10}}}
profile['profile_path']=str(dest/'profile.json');(dest/'profile.json').write_text(json.dumps(profile,indent=2))
print(dest/'profile.json')
