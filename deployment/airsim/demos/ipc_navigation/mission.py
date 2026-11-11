"""Real IPC client; initial gate reuses control acceptance before forest navigation."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ipc_control'))
import mission
# Explicit real-source readiness; retain recovery and scoring implementation.
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
class NativeMission(mission.Mission):
    def __init__(self):
        super().__init__()
        self.input_counts={'odom':0,'cloud':0};self.input_stamps={}
        for key,kind,topic in [('odom',Odometry,'/drone0/navigation/odom'),('cloud',PointCloud2,'/drone0/navigation/points')]:
            self.create_subscription(kind,topic,lambda m,k=key:(self.input_counts.__setitem__(k,self.input_counts[k]+1),self.input_stamps.__setitem__(k,m.header.stamp.sec+m.header.stamp.nanosec/1e9)),mission.qos_profile_sensor_data)
        self.result['scope']='Real IPC native control gate; not forest acceptance'
        self.create_subscription(Bool,'/drone0/ipc/ready',lambda m:setattr(self,'source_ready',m.data),1)
    def preflight_probe(self):
        self.wait(lambda: all(v>=3 for v in self.input_counts.values()) and all(0<=self.get_clock().now().nanoseconds/1e9-self.input_stamps[k]<=limit for k,limit in [('odom',.2),('cloud',.5)]),30)
        self.result['input_counts_before_arming']=dict(self.input_counts)
        super().preflight_probe()
mission.Mission=NativeMission
if __name__=='__main__':mission.main()
