import rclpy
from tf2_msgs.msg import TFMessage
from as2_msgs.msg import PlatformInfo
rclpy.init()
n=rclpy.create_node('aerohub_dds_probe')
for name,kind in [('tf',TFMessage),('platform',PlatformInfo)]:
 print('creating',name,flush=True)
 p=n.create_publisher(kind,name,10)
 p.publish(kind())
 print('published',name,flush=True)
n.destroy_node();rclpy.shutdown()
