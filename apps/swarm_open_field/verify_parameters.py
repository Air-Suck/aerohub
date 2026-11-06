"""Read back native runtime configuration over DDS; never change parameters."""
import json,time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType


def main():
    rclpy.init();node=rclpy.create_node('swarm_readonly_parameters');results={}
    names=['optimization.vehicle_count','optimization.swarm_clearance','global_goal.swarm_scale',
           'global_goal.altitude_from_message','manager.drone_id','use_sim_time']
    try:
        for i in range(3):
            client=node.create_client(GetParameters,f'/drone_{i}_ego_planner_node/get_parameters')
            assert client.wait_for_service(timeout_sec=15),'native planner not found'
            request=GetParameters.Request();request.names=names;future=client.call_async(request)
            until=time.monotonic()+5
            while not future.done() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
            assert future.done(),'parameter response timeout'
            values=[]
            for p in future.result().values:
                values.append({ParameterType.PARAMETER_BOOL:p.bool_value,ParameterType.PARAMETER_INTEGER:p.integer_value,
                               ParameterType.PARAMETER_DOUBLE:p.double_value}.get(p.type))
            results[f'drone{i}']=dict(zip(names,values))
            assert values==[3,2.0,2.0,True,i,True],values
        Path('/results/native-parameters.json').write_text(json.dumps({'passed':True,'vehicles':results},indent=2)+'\n')
        print(json.dumps(results,indent=2))
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
