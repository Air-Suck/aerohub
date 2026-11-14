import json,pathlib,sys,copy
root=pathlib.Path.cwd();sys.path.insert(0,str(root/'simulator/projectairsim'))
import commentjson
base=root/'log/projectairsim/package-course-wmm'
profile=json.loads((base/'profile.json').read_text())
source=pathlib.Path(profile['scene_config']);dest=root/'log/projectairsim/diffphys-course-config'
dest.mkdir(parents=True,exist_ok=True)
scene=commentjson.loads(source.read_text());robot=commentjson.loads((source.parent/scene['actors'][0]['robot-config']).read_text())
# Use the native sensor camera, not the stabilized Chase observer camera.
front=copy.deepcopy(next(s for s in robot['sensors'] if s['id']=='DownCamera'))
front['capture-interval']=1/15
front['gimbal']={'lock-roll':False,'lock-pitch':False,'lock-yaw':False}
front['id']='diffphys_front';front['origin']={'xyz':'0.5 0 -0.1','rpy-deg':'0 20 0'}
for c in front['capture-settings']:
 c['capture-enabled']=c['image-type']==1
 c['streaming-enabled']=False
 c['pixels-as-float']=c['image-type']==1
 c['compress']=False
 c.update(width=640,height=480,**{'fov-degrees':91})
if not any(c['image-type']==1 for c in front['capture-settings']):
 c=copy.deepcopy(front['capture-settings'][0]);c.update(**{'image-type':1,'capture-enabled':True});front['capture-settings'].append(c)
for s in robot['sensors']:
 if s['type']=='camera':
  s['enabled']=s['id']=='Chase'
  for capture in s['capture-settings']:
   capture['capture-enabled']=False
   capture['streaming-enabled']=s['id']=='Chase' and capture['image-type']==0
robot['sensors'].append(front)
scene['actors'][0]['robot-config']='robot_diffphys.jsonc'
(dest/'robot_diffphys.jsonc').write_text(json.dumps(robot,indent=2))
(dest/'scene_diffphys.jsonc').write_text(json.dumps(scene,indent=2))
profile['adapter']['environment']['PROJECTAIRSIM_DDS_PROFILE']=str(root/'deployment/airsim/demos/projectairsim_course/diffphys-dds.xml')
profile['scene_config']=str(dest/'scene_diffphys.jsonc');profile['sensor_options']={'Drone1':{'diffphys_front':{'enabled':True,'position':[0.5,0,-.1],'rotation':[0,20,0],'width':640,'height':480,'fov':91,'captures':['depth_planar']},'IMU1':{'enabled':True},'lidar1':{'type':'lidar','enabled':True,'position':[0,0,.2],'channels':16,'range':100,'points_per_second':100000,'rotation_hz':10}}}
profile['sensor_options']['Drone1'].pop('lidar1',None)
profile['profile_path']=str(dest/'profile.json');(dest/'profile.json').write_text(json.dumps(profile,indent=2))
print(dest/'profile.json')
