"""Replay the frozen native tail through real Adapter and AS2 (no native process).

Normal replay gives both sources the same epoch translation. The negative
case additionally ages the candidate by 200 ms, so it is un-dispatchable while
the already accepted path is still executing (not already settling).
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from quadrotor_msgs.msg import GoalSet
from traj_utils.msg import PolyTraj
from uav_ego_planner_adapter.conversion import Polynomial, nanoseconds, set_ns
from test_terminal_hold import DATA,message
from tail_reference_check import compare_references,segment_rows
from verify_split import SplitHarness,project


class Replay(SplitHarness):
    def __init__(self):
        super().__init__()
        self.native_goals=[];self.sent=[]
        self.create_subscription(GoalSet,'/drone1/ego/native/goal',lambda m:self.native_goals.append(m),
                                 self.goals_by_vehicle['drone1'].qos_profile)
        self.output=self.create_publisher(PolyTraj,'/drone1/ego_planner/planning/trajectory',10)
        self.heartbeat=self.create_publisher(Empty,'/drone1/ego_planner/planning/heartbeat',10)
        self.create_timer(.02,lambda:self.heartbeat.publish(Empty()))
        initial=Polynomial(message(DATA['previous']),1,.2415,.3675).sample(DATA['previous']['start_ns'])[0]
        self.state['drone1']['p']=[initial[0],initial[1]-4,initial[2]]

    def run_replay(self,mode):
        self.until(lambda:self.status['drone1'].get('state')=='WAITING' and self.cancels['drone1'].service_is_ready()
                   and self.goals_by_vehicle['drone1'].get_subscription_count()>0 and self.output.get_subscription_count()>=2,15)
        self.settle(1)
        self.send('drone1');self.until(lambda:bool(self.native_goals),10)
        shift=self.get_clock().now().nanoseconds-DATA['previous']['start_ns']
        old=message(DATA['previous']);new=message(DATA['candidate'])
        set_ns(old.start_time,DATA['previous']['start_ns']+shift)
        set_ns(new.start_time,DATA['candidate']['start_ns']+shift)
        if mode=='late':set_ns(new.start_time,nanoseconds(new.start_time)-200_000_000)
        self.sent=[old,new];self.output.publish(old)
        self.until(lambda:self.status['drone1'].get('revision',0)>=1 and len(self.refs['drone1'])>=5,5)
        publish_after=nanoseconds(new.start_time)+(120_000_000 if mode=='late' else 5_000_000)
        self.until(lambda:self.get_clock().now().nanoseconds>=publish_after,5)
        before=self.status['drone1']['revision']
        self.output.publish(new)
        self.until(lambda:self.status['drone1'].get('state') in ('SUCCEEDED','FAILED') and self.status['drone1'].get('stop_confirmed'),8)
        state=self.status['drone1']
        if mode=='normal':
            self.check('replay_completed',state['state']=='SUCCEEDED')
            self.check('changed_tail_actually_submitted',state['revision']==before+1)
            self.check('terminal_hold_used',state['counts'].get('terminal_hold_segments')==1)
            self.check('not_ignored_as_equivalent',state['counts'].get('redundant_final_tails',0)==0)
        else:
            self.check('late_replay_rejected',state['state']=='FAILED' and state['reason'].startswith('INVALID_COVERAGE'))
            self.check('late_not_submitted',state['revision']==before and not state['counts'].get('terminal_hold_segments'))
        sources=[Polynomial(m,1,.2415,.3675) for m in self.sent]
        error,holds=compare_references(sources,self.refs['drone1'],self.segments['drone1'],'drone1',[0,4,0],DATA['goal'])
        self.pva_errors['drone1']=error;self.hold_reference_counts['drone1']=holds
        self.check('actual_reference_pva',error<=1e-4)
        if mode=='normal':self.check('as2_consumed_terminal_hold',holds>=2)
        self.check('original_limits',self.pva_max[0]<=.2415 and self.pva_max[1]<=.3675)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path);parser.add_argument('mode',choices=('normal','late'))
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    config=yaml.safe_load((project/'src/planner/ego_planner_v2/ros2_adapter/tests/data/dual.yaml').read_text())
    (out/'config.yaml').write_text(yaml.safe_dump(config))
    rclpy.init();node=Replay();processes=[];streams=[];passed=False;error=''
    commands=[['ros2','run','uav_ego_planner_adapter','ego_as2_adapter','--ros-args','-r','__ns:=/drone1',
               '-p',f'resolved_config:={out}/config.yaml','-p','vehicle:=drone1'],
              ['ros2','run','uav_follow_path_execution','follow_path_extended_node','--ros-args',
               '--params-file',str(Path(get_package_share_directory('as2_behaviors_motion'))/'follow_path_behavior/config/config_default.yaml'),
               '-r','__ns:=/drone1','-p','plugin_name:=follow_path_plugin_position','-p','vehicle_id:=drone1',
               '-p','follow_path_speed:=0.2415','-p','follow_path_threshold:=0.2','-p','tf_timeout_threshold:=0.1']]
    (out/'commands.json').write_text(json.dumps(commands,indent=2))
    try:
        for i,cmd in enumerate(commands):
            f=(out/f'process-{i}.log').open('w');streams.append(f)
            processes.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
        node.run_replay(args.mode);passed=True
    except Exception as exc:error=repr(exc)
    finally:
        if node.cancels['drone1'].service_is_ready():node.cancels['drone1'].call_async(Trigger.Request())
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and not node.status['drone1'].get('stop_confirmed'):rclpy.spin_once(node,timeout_sec=.01)
        cleanup=[]
        for process in processes:
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
            deadline=time.monotonic()+5
            while time.monotonic()<deadline and process.poll() is None:rclpy.spin_once(node,timeout_sec=.01)
            if process.poll() is None:os.killpg(process.pid,signal.SIGKILL);process.wait();cleanup.append('forced kill')
            else:cleanup.append(process.returncode)
            node.settle(.3)
        for f in streams:f.close()
        result=dict(passed=passed and all(c==0 for c in cleanup),error=error,mode=args.mode,checks=node.checks,
            status=node.status['drone1'],cleanup=cleanup,original_time_pva_error=node.pva_errors,
            hold_reference_counts=node.hold_reference_counts,scope='recorded native messages replayed through real Adapter and AS2; synthetic controller')
        for name,data in [('result',result),('references',node.refs['drone1']),('segments',segment_rows(node.segments['drone1'])),
                          ('sent',[dict(start_ns=nanoseconds(m.start_time),duration=list(m.duration),
                              coef_x=list(m.coef_x),coef_y=list(m.coef_y),coef_z=list(m.coef_z)) for m in node.sent])]:
            (out/(name+'.json')).write_text(json.dumps(data,indent=2))
        node.destroy_node();rclpy.shutdown();print(json.dumps(result,indent=2))
        return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
