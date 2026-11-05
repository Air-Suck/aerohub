import rclpy
from tf2_ros import Buffer,TransformListener,TransformBroadcaster,StaticTransformBroadcaster
rclpy.init(); n=rclpy.create_node('aerohub_tf_probe')
for name,fn in [('listener',lambda:TransformListener(Buffer(),n)),('broadcaster',lambda:TransformBroadcaster(n)),('static',lambda:StaticTransformBroadcaster(n))]:
 print(name,flush=True);obj=fn();print('ok',flush=True)
n.destroy_node();rclpy.shutdown()
