"""Read-only startup probe using one ROS context for discovery and parameters."""
import argparse
import json
import time

import rclpy
from rcl_interfaces.srv import GetParameters


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tf-scope',choices=('shared','per-vehicle'),default='shared')
    parser.add_argument('required',nargs='+')
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('onboard_image_probe')
    clients={name:node.create_client(GetParameters,'/drone0/'+name+'/get_parameters') for name in args.required}
    def wait(predicate,seconds):
        deadline=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>deadline:raise TimeoutError('startup discovery/parameter response timeout')
            rclpy.spin_once(node,timeout_sec=.05)
    try:
        wait(lambda:all(c.service_is_ready() for c in clients.values()),25)
        futures={}
        for name,client in clients.items():
            request=GetParameters.Request();request.names=['use_sim_time'];futures[name]=client.call_async(request)
        wait(lambda:all(f.done() for f in futures.values()),10)
        parameters={name:future.result().values[0].bool_value for name,future in futures.items()}
        assert all(parameters.values()),parameters
        nodes=sorted(ns.rstrip('/')+'/'+name for name,ns in node.get_node_names_and_namespaces())
        assert all('/drone0/'+name in nodes for name in args.required),nodes
        topics=sorted(name for name,_ in node.get_topic_names_and_types())
        if args.tf_scope=='per-vehicle':
            wait(lambda:all(t in dict(node.get_topic_names_and_types()) for t in ('/drone0/tf','/drone0/tf_static')),5)
            topics=sorted(name for name,_ in node.get_topic_names_and_types())
            assert '/tf' not in topics and '/tf_static' not in topics,topics
        print(json.dumps({'passed':True,'nodes':nodes,'use_sim_time':parameters,'tf_scope':args.tf_scope,'topics':topics},indent=2),flush=True)
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
