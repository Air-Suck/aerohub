import os,subprocess,time,signal
from pathlib import Path
base=Path(__file__).resolve().parent
# Installed diagnostics live directly in the chosen AS2 root.
root=str(base/'install')
command=[root+'/as2_state_estimator/lib/as2_state_estimator/as2_state_estimator_node','--ros-args','--params-file',root+'/as2_state_estimator/share/as2_state_estimator/config/state_estimator_default.yaml','--params-file',root+'/as2_state_estimator/share/as2_state_estimator/plugins/raw_odometry/config/plugin_default.yaml','-r','__ns:=/aerohub_probe','-p','plugin_name:=raw_odometry','-p','use_sim_time:=false','-p','use_gps:=false','-p','set_origin_on_start:=true']
with open(base/'estimator-smoke.log','w') as stream:
 p=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  time.sleep(5)
  if p.poll() is not None:raise RuntimeError('estimator exited '+str(p.returncode))
  print('AS2 estimator alive after 5 seconds')
 finally:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
print('process cleaned up',p.returncode)
