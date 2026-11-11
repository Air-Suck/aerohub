"""Supervise one PC compute container per PC UAV; copy startup calibration/config once with hashes."""
import hashlib,json,os,shlex,signal,subprocess,threading,time
from pathlib import Path
import yaml


def container_present(name):
    return subprocess.run(['docker','inspect',name],capture_output=True).returncode==0


class StartupTransfer:
    """Copy board-resolved startup files into the read-only directory shared by PC workers."""
    def __init__(self,out,remote,board,hdc):
        self.out=out;self.remote=remote;self.board=board;self.hdc=hdc
        self.shared=out/'shared';(self.shared/'resolved').mkdir(parents=True)
        self.stopping=threading.Event();self.thread=None
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
            while not self.stopping.is_set():
                code='import pathlib,json;p=pathlib.Path('+repr(self.remote+'/results/nav-fleet.yaml')+');print(json.dumps([str(p)] if p.is_file() else []))'
                paths=json.loads(self.board('python3 -c '+shlex.quote(code),timeout=15))
                if paths:
                    records.append(self.copy(paths[0],self.shared/'nav-fleet.yaml'));break
                self.stopping.wait(.5)
            local_resolved=self.out/'resolved'
            while not self.stopping.is_set():
                files=sorted(local_resolved.glob('*.yaml')) if local_resolved.is_dir() else []
                if files:
                    target=self.shared/'resolved'/files[0].name
                    target.write_bytes(files[0].read_bytes());yaml.safe_load(target.read_text())
                    records.append({'local':str(files[0]),'target':str(target),'sha256':hashlib.sha256(target.read_bytes()).hexdigest()})
                    break
                remote_code='import pathlib,json;p=pathlib.Path('+repr(self.remote+'/results/resolved')+');print(json.dumps([str(x) for x in p.glob("*.yaml")] if p.is_dir() else []))'
                paths=json.loads(self.board('python3 -c '+shlex.quote(remote_code),timeout=15))
                if paths:
                    remote=paths[0];records.append(self.copy(remote,self.shared/'resolved'/Path(remote).name));break
                self.stopping.wait(.5)
            (self.out/'startup-transfers.json').write_text(json.dumps(records,indent=2)+'\n')
        except Exception as exc:(self.out/'transfer-failure.txt').write_text(str(exc))
    def start_transfer(self):
        self.thread=threading.Thread(target=self.transfer,name='swarm-config-transfer',daemon=True);self.thread.start()
    def stop_transfer(self):
        self.stopping.set()
        if self.thread:self.thread.join(timeout=16)


class ComputeWorker(threading.Thread):
    """Own one PC UAV container for the lifetime of one supervisor thread."""
    def __init__(self, owner, vehicle):
        super().__init__(name=f"swarm-pc-{vehicle['name']}", daemon=False)
        self.owner = owner
        self.vehicle = vehicle
        self.container = owner.prefix + "compute" + str(vehicle["index"])
        self.process = None
        self.thread_id = None
        self.returncode = None
        self.error = None
        self.ready = threading.Event()

    def command(self, root):
        vehicle = self.vehicle["name"]
        return [
            "docker", "run", "--rm", "--init", "--network", "host",
            "--name", self.container,
            "-e", "ROS_DOMAIN_ID=91", "-e", "ROS_LOCALHOST_ONLY=0",
            "-e", "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
            "-e", "FASTRTPS_DEFAULT_PROFILES_FILE=/run/dds.xml",
            "-e", "COMPUTE_ROOT=/run/node",
            "-e", "COMPUTE_VEHICLE=" + vehicle,
            "-e", "CATORA_COMPUTE_ID=pc-" + vehicle,
            "-e", "AEROHUB_SWARM_VEHICLE_COUNT=" + str(self.owner.vehicle_count),
            "-e", "AEROHUB_SWARM_SCENARIO=" + self.owner.scenario,
            "-e", "AEROHUB_SWARM_CENTER=" + self.owner.center_vehicle,
            "-v", str(root) + ":/run/node",
            "-v", str(self.owner.shared) + ":/run/resolved:ro",
            "-v", str(self.owner.out / "pc-dds.xml") + ":/run/dds.xml:ro",
            self.owner.image,
        ]

    def run(self):
        self.thread_id = threading.get_ident()
        vehicle = self.vehicle["name"]
        root = self.owner.out / "compute" / vehicle
        try:
            root.mkdir(parents=True, exist_ok=True)
            (root / "planner.yaml").write_bytes((self.owner.app / "planner.yaml").read_bytes())
            local_fleet = {"vehicles": {vehicle: self.owner.fleet["vehicles"][vehicle]}}
            (root / "fleet.yaml").write_text(yaml.safe_dump(local_fleet))
            with (root / "container.log").open("w") as log:
                self.process = subprocess.Popen(
                    self.command(root), stdout=log, stderr=subprocess.STDOUT, start_new_session=True
                )
                self.ready.set()
                self.returncode = self.process.wait()
                if not self.owner.stopping.is_set():
                    self.error = f"{self.container} exited before stop with code {self.returncode}"
        except BaseException as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            self.ready.set()

    def kill_client(self):
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def record(self):
        return {
            "vehicle": self.vehicle["name"], "placement": "pc",
            "thread_name": self.name, "thread_id": self.thread_id,
            "container": self.container,
            "docker_pid": None if self.process is None else self.process.pid,
            "returncode": self.returncode,
            "error": self.error,
        }


class Workers(StartupTransfer):
    def __init__(
        self, out, app, prefix, image, remote, board, hdc,
        pc_vehicles, fleet, vehicle_count, scenario, center_vehicle,
        startup_timeout=30.0, stop_timeout=8,
    ):
        super().__init__(out, remote, board, hdc)
        self.app = app
        self.prefix = prefix
        self.image = image
        self.fleet = fleet
        self.vehicle_count = vehicle_count
        self.scenario = scenario
        self.center_vehicle = center_vehicle
        self.startup_timeout = startup_timeout
        self.stop_timeout = stop_timeout
        self.workers = [ComputeWorker(self, vehicle) for vehicle in pc_vehicles]
        self.names = [worker.container for worker in self.workers]
        self.final = None

    def start(self):
        for worker in self.workers:
            worker.start()
        deadline = time.monotonic() + self.startup_timeout
        for worker in self.workers:
            if not worker.ready.wait(max(0, deadline - time.monotonic())):
                raise RuntimeError("PC worker thread startup timed out: " + worker.name)
        while time.monotonic() < deadline:
            if any(worker.error for worker in self.workers):
                raise RuntimeError("PC worker failed during startup")
            running = [
                subprocess.run(
                    ["docker", "inspect", "--format", "{{.State.Running}}", worker.container],
                    capture_output=True, text=True,
                ).stdout.strip() == "true"
                for worker in self.workers
            ]
            if all(running):
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("PC worker containers did not become ready")
        records = [worker.record() for worker in self.workers]
        if len({record["thread_id"] for record in records}) != len(records):
            raise RuntimeError("PC UAV workers do not have independent supervisor threads")
        if len({record["docker_pid"] for record in records}) != len(records):
            raise RuntimeError("PC UAV workers do not own independent Docker processes")
        (self.out / "pc-worker-topology.json").write_text(
            json.dumps(records, indent=2) + "\n"
        )
        self.start_transfer()

    def stop(self):
        """Stop every container, join every started thread and remove leftovers; safe before start()."""
        self.stop_transfer()
        for worker in self.workers:
            subprocess.run(
                ["docker", "stop", "--timeout", str(self.stop_timeout), worker.container],
                capture_output=True,
            )
        for worker in self.workers:
            if worker.ident is not None:
                worker.join(timeout=self.stop_timeout + 4)
        for worker in self.workers:
            if worker.is_alive() or container_present(worker.container):
                subprocess.run(["docker", "rm", "--force", worker.container], capture_output=True)
                worker.kill_client()
                worker.join(timeout=5)
        records = []
        for worker in self.workers:
            record = worker.record()
            record["thread_alive"] = worker.is_alive()
            record["container_present"] = container_present(worker.container)
            records.append(record)
        self.final = records
        (self.out / "pc-worker-final.json").write_text(
            json.dumps(records, indent=2) + "\n")
        return records

    def failures(self):
        return [
            record for record in self.final or []
            if record["error"] or record["thread_alive"] or record["container_present"]
        ]
