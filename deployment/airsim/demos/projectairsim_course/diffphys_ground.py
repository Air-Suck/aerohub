"""Disarmed DiffPhys real-input readiness probe; no motion commands."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.msg import PlatformInfo
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
rclpy.init();n=Node('diffphys_ground_probe');ready=[];platform=[];images=[]
n.create_subscription(Bool,'/drone0/diffphys/ready',ready.append,1)
n.create_subscription(PlatformInfo,'/drone0/platform/info',platform.append,qos_profile_sensor_data)
n.create_subscription(Image,'/drone0/sensor_measurements/diffphys_front/depth_planar/image',lambda m:images.__setitem__(slice(None),[m]),qos_profile_sensor_data)
end=time.monotonic()+45
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.05)
 if platform and platform[-1].armed:raise RuntimeError('ground probe found armed vehicle')
 if ready and ready[-1].data and platform:break
if images:
 image=images[-1]
 Path('/evidence/ground-depth.bin').write_bytes(bytes(image.data))
 Path('/evidence/ground-depth.json').write_text(json.dumps(dict(width=image.width,height=image.height,step=image.step,encoding=image.encoding,frame=image.header.frame_id)))
success=bool(ready and ready[-1].data and platform and not platform[-1].armed)
Path('/evidence/result.json').write_text(json.dumps(dict(success=success,motion_requested=False,disarmed=bool(platform and not platform[-1].armed),scope='real camera/state/model service ready, not flight'),indent=2))
n.destroy_node();rclpy.shutdown()
if not success:raise SystemExit(1)
