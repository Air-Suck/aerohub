import subprocess,os,time,signal
p=subprocess.Popen(['python3','run_as2.py'],stdout=open('full-smoke.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
try:
 time.sleep(12)
 if p.poll() is not None:raise RuntimeError('AS2 launcher exited '+str(p.returncode))
 result=subprocess.run(['ros2','node','list','--no-daemon'],capture_output=True,text=True,timeout=10)
 print(result.stdout,flush=True)
 required=['/drone0/platform','/drone0/state_estimator','/drone0/controller_manager','/drone0/TakeoffBehavior','/drone0/LandBehavior']
 if not all(n in result.stdout.splitlines() for n in required):raise RuntimeError('missing AS2 nodes')
 print('AS2 full startup passed',flush=True)
finally:
 if p.poll() is None:p.terminate()
 try:p.wait(timeout=10)
 except subprocess.TimeoutExpired:p.kill();p.wait()
 print('launcher exit',p.returncode,flush=True)
