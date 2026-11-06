"""Host lifecycle for three logical compute boards; HDC only transfers startup config."""
import hashlib,json,os,shlex,subprocess,threading,time
from pathlib import Path
import yaml


class HybridWorkers:
    def __init__(self,output,app,prefix,image,remote,board,hdc):
        self.output=output;self.app=app;self.prefix=prefix;self.image=image;self.remote=remote;self.board=board;self.hdc=hdc
        self.names=[prefix+'compute'+str(i) for i in range(1,4)]
        self.stopping=threading.Event();self.thread=None;self.transfer=None;self.error=None
        self.shared=output/'hybrid-resolved';self.shared.mkdir(exist_ok=True)
        for name in self.names:
            if subprocess.run(['docker','inspect',name],capture_output=True).returncode==0:raise RuntimeError('existing compute node: '+name)

    def start(self):
        fleet=yaml.safe_load((self.app/'fleet.yaml').read_text())
        deployment=yaml.safe_load((self.app/'deployment.yaml').read_text())
        for index,name in enumerate(self.names,1):
            vehicle='drone'+str(index);folder=self.output/'compute'/vehicle;folder.mkdir(parents=True)
            (folder/'fleet.yaml').write_text(yaml.safe_dump({'vehicles':{vehicle:fleet['vehicles'][vehicle]}}))
            (folder/'deployment.yaml').write_text(yaml.safe_dump({vehicle:deployment[vehicle]}))
            subprocess.run(['docker','run','--rm','--detach','--init','--network','host','--name',name,
                '-e','ROS_DOMAIN_ID=91','-e','ROS_LOCALHOST_ONLY=0','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
                '-e','FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml','-e','COMPUTE_ROOT=/run/node',
                '-e','COMPUTE_VEHICLE='+vehicle,'-e','CATORA_COMPUTE_ID=pc-'+vehicle,
                '-v',str(folder)+':/run/node','-v',str(self.shared)+':/run/resolved:ro',
                '-v',str(self.output/'pc-dds.xml')+':/run/dds.xml:ro',self.image],check=True,stdout=subprocess.DEVNULL)
        self.thread=threading.Thread(target=self.transfer_config,daemon=True);self.thread.start()

    def transfer_config(self):
        try:
            while not self.stopping.is_set():
                code='import pathlib,json; print(json.dumps([str(p) for p in pathlib.Path('+repr(self.remote+'/results/resolved')+').glob("*.yaml")]))'
                paths=json.loads(self.board('python3 -c '+shlex.quote(code),timeout=15))
                if paths:
                    remote=paths[0];part=self.shared/'config.part';started=time.monotonic()
                    self.hdc('file','recv',remote,str(part))
                    code='import hashlib; print(hashlib.sha256(open('+repr(remote)+',"rb").read()).hexdigest())'
                    expected=self.board('python3 -c '+shlex.quote(code)).strip()
                    assert hashlib.sha256(part.read_bytes()).hexdigest()==expected
                    config=yaml.safe_load(part.read_text());target=self.shared/(config['task']['task_id']+'.yaml');part.replace(target)
                    self.transfer={'remote':remote,'sha256':expected,'wall_ms':(time.monotonic()-started)*1000,'task_id':config['task']['task_id']}
                    (self.output/'resolved-transfer.json').write_text(json.dumps(self.transfer,indent=2)+'\n')
                    return
                self.stopping.wait(1)
        except Exception as exc:
            self.error=str(exc);(self.output/'config-transfer-error.txt').write_text(self.error)

    def stop(self):
        self.stopping.set()
        if self.thread:self.thread.join(timeout=16)
        for index,name in enumerate(self.names,1):
            log=self.output/'compute'/('drone'+str(index))/'container.log'
            log.parent.mkdir(parents=True,exist_ok=True)
            with log.open('w') as stream:subprocess.run(['docker','logs',name],stdout=stream,stderr=subprocess.STDOUT)
            subprocess.run(['docker','stop','--timeout','8',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
