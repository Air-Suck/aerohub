"""Board-owned test gate; exact binary, namespace, parent, domain and start-time checks."""
import json
import os
from pathlib import Path
import signal
import rclpy
from ament_index_python.packages import get_package_prefix
from fault_injector_base import Injector

ROOT=Path(__file__).resolve().parent

def process_identity(path):
    argv=(path/'cmdline').read_bytes().decode().split('\0')
    fields=(path/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=int(path.name),argv=argv,parent=int(fields[1]),ticks=fields[19],
                domain=[s.decode() for s in (path/'environ').read_bytes().split(b'\0') if s.startswith(b'ROS_DOMAIN_ID=')])

def owned(identity, binary, parent, domain):
    return (identity['argv'][0]==binary and '__ns:=/drone0' in identity['argv'] and
            identity['parent']==parent and identity['domain']==['ROS_DOMAIN_ID='+str(domain)])

class BoardInjector(Injector):
    def __init__(self):
        self.config=json.loads((ROOT/'config.json').read_text())
        super().__init__()

    def gate(self,key,req,res):
        if self.config['fault_case'] != key:
            res.success=False;res.message='This session does not authorize '+key;return res
        return super().gate(key,req,res)

    def kill(self,req,res):
        if self.config['fault_case'] != 'planner_exit':
            res.success=False;res.message='This session does not authorize process exit';return res
        binary=str(Path(get_package_prefix('ego_planner'))/'lib/ego_planner/ego_planner_node')
        if not Path(binary).resolve().is_relative_to(Path(self.config['candidate']).resolve()):
            res.success=False;res.message='Planner binary is outside candidate';return res
        candidates=[]
        for path in Path('/proc').glob('[0-9]*'):
            try:
                value=process_identity(path)
                if owned(value,binary,os.getppid(),self.config['domain']):candidates.append(value)
            except (OSError,ValueError,IndexError,UnicodeError):pass
        if len(candidates)!=1:
            res.success=False;res.message='Expected one owned planner: '+repr(candidates);return res
        value=candidates[0]
        if process_identity(Path('/proc')/str(value['pid'])) != value:
            res.success=False;res.message='Process identity changed';return res
        os.kill(value['pid'],signal.SIGKILL)
        self.event(kind='planner_exit',signal='SIGKILL',run_id=self.config['run_id'],**value)
        res.success=True;res.message=json.dumps(self.events[-1]);return res

if __name__=='__main__':
    rclpy.init();node=BoardInjector()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
