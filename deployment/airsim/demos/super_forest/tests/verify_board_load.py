"""Run from the installed AS2 candidate; no goals or arming requests."""
import signal
import subprocess
import sys
import time
from pathlib import Path

root=Path.cwd()
sys.path.insert(0,str(root/'super_app'))
import msgpack
from uav_super_adapter.node import SuperAdapter
from uav_planner_interfaces.action import FollowTimedPath
from rclpy.type_support import check_for_type_support
check_for_type_support(FollowTimedPath)
assert msgpack.unpackb(msgpack.packb({'ok':1}),raw=False)=={'ok':1}
print('PASS: board Adapter, FollowTimedPath and pure-Python msgpack')
with (root/'runtime/super-load.log').open('w') as log:
    child=subprocess.Popen([str(root/'install/super_planner/lib/super_planner/fsm_node'),
        '--ros-args','-p','use_sim_time:=true','-p','config_name:='+str(root/'super_app/planner.yaml')],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        time.sleep(4)
        assert child.poll() is None,'Native SUPER exited; inspect runtime/super-load.log'
        print('PASS: native OHOS SUPER loaded for four seconds; no motion')
    finally:
        if child.poll() is None:child.send_signal(signal.SIGINT)
        try:child.wait(timeout=8)
        except subprocess.TimeoutExpired:child.kill();child.wait()
    print('Native SUPER exit',child.returncode)
