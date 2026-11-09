"""Own two PC compute containers and copy startup calibration/config once with hashes."""
import hashlib,json,os,shlex,subprocess,threading,time
from pathlib import Path
import yaml

class Workers:
    def __init__(self,out,app,prefix,image,remote,board,hdc):
        self.out=out;self.app=app;self.prefix=prefix;self.image=image;self.remote=remote;self.board=board;self.hdc=hdc
        self.shared=out/'shared';(self.shared/'resolved').mkdir(parents=True)
        self.names=[prefix+f'compute{i}' for i in (1,2)];self.stopping=threading.Event();self.thread=None
    def start(self):
        fleet=yaml.safe_load((self.app/'fleet.yaml').read_text())
        for i,name in enumerate(self.names,1):
            vehicle=f'drone{i}';root=self.out/'compute'/vehicle;root.mkdir(parents=True)
            (root/'planner.yaml').write_bytes((self.app/'planner.yaml').read_bytes())
            (root/'fleet.yaml').write_text(yaml.safe_dump({'vehicles':{vehicle:fleet['vehicles'][vehicle]}}))
            subprocess.run(['docker','run','--rm','--detach','--init','--network','host','--name',name,
                '-e','ROS_DOMAIN_ID=91','-e','ROS_LOCALHOST_ONLY=0','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
                '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml','-e','COMPUTE_ROOT=/run/node',
                '-e','COMPUTE_VEHICLE='+vehicle,'-e','CATORA_COMPUTE_ID=pc-'+vehicle,
                '-v',str(root)+':/run/node','-v',str(self.shared)+':/run/resolved:ro',
                '-v',str(self.out/'pc-dds.xml')+':/run/dds.xml:ro',self.image],check=True,stdout=subprocess.DEVNULL)
        self.thread=threading.Thread(target=self.transfer,daemon=True);self.thread.start()
    def copy(self,remote,target):
        partial=target.with_suffix('.part');self.hdc('file','recv',remote,str(partial))
        code='import hashlib;print(hashlib.sha256(open('+repr(remote)+',"rb").read()).hexdigest())'
        expected=self.board('python3 -c '+shlex.quote(code))
        assert hashlib.sha256(partial.read_bytes()).hexdigest()==expected
        yaml.safe_load(partial.read_text());partial.replace(target)
        return {'remote':remote,'target':str(target),'sha256':expected}
    def transfer(self):
        records=[]
        try:
            for relative,target in [('nav-fleet.yaml',self.shared/'nav-fleet.yaml'),('resolved',None)]:
                while not self.stopping.is_set():
                    code='import pathlib,json;p=pathlib.Path('+repr(self.remote+'/results/'+relative)+');print(json.dumps([str(x) for x in p.glob("*.yaml")] if p.is_dir() else ([str(p)] if p.is_file() else [])))'
                    paths=json.loads(self.board('python3 -c '+shlex.quote(code),timeout=15))
                    if paths:
                        remote=paths[0];records.append(self.copy(remote,target or self.shared/'resolved'/Path(remote).name));break
                    self.stopping.wait(.5)
            (self.out/'startup-transfers.json').write_text(json.dumps(records,indent=2)+'\n')
        except Exception as exc:(self.out/'transfer-failure.txt').write_text(str(exc))
    def stop(self):
        self.stopping.set()
        if self.thread:self.thread.join(timeout=16)
        for i,name in enumerate(self.names,1):
            root=self.out/'compute'/f'drone{i}';root.mkdir(parents=True,exist_ok=True)
            with (root/'container.log').open('w') as log:subprocess.run(['docker','logs',name],stdout=log,stderr=subprocess.STDOUT)
            subprocess.run(['docker','stop','--timeout','8',name],capture_output=True)
