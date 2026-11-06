"""Ground-only AirSim/PX4 handshake loop. Does not deploy or contact RoboPi."""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
from run import APP,ROOT,PREFIX,NAMES,manager,cleanup
from sim_startup import launch_vehicles
sys.path.insert(0,str(ROOT/'scripts'))
from robopi import profile

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--delay',type=float,default=.1);p.add_argument('--runs',type=int,default=3);p.add_argument('--output',type=Path,required=True);p.add_argument('--image')
a=p.parse_args();root=a.output.resolve();root.mkdir(parents=True,exist_ok=True)
image=a.image or os.environ['FLIGHT_BOARD_IMAGE'];results=[]
for attempt in range(a.runs):
    output=root/str(attempt);output.mkdir();relays=[]
    if any(subprocess.run(['docker','inspect',n],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0 for n in NAMES):raise RuntimeError('existing app resource')
    profile(output/'dds.xml',os.environ['PC_IP'])
    started=time.monotonic();record={'attempt':attempt,'delay':a.delay,'image':image}
    try:
        subprocess.run(manager('start',output)+['--timeout','90'],check=True,stdout=subprocess.DEVNULL)
        relays=launch_vehicles(image,APP,output,PREFIX,output/'dds.xml',a.delay)
        record['passed']=True
    except Exception as exc:record.update(passed=False,error=str(exc))
    finally:
        for relay in relays:relay.close()
        cleanup(output)
    record['elapsed']=time.monotonic()-started;results.append(record)
    (root/'result.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(record),flush=True)
raise SystemExit(0 if all(r['passed'] for r in results) else 1)
