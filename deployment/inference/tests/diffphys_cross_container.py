"""Build isolated ROS overlays and verify a CUDA worker from two fixed containers."""

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
from urllib.request import ProxyHandler, build_opener


ROOT = Path(__file__).resolve().parents[3]
GROUND = "osrf/ros:humble-desktop-full-jammy"
ONBOARD = "onboard_computer:v0.1.0"


def run(command, timeout=180):
    return subprocess.check_output(command, stderr=subprocess.STDOUT, text=True, timeout=timeout)


def main():
    report = ROOT / "log/diffphys/model-chain" / str(os.getpid())
    report.mkdir(parents=True)
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    domain = 210 + os.getpid() % 20
    name = "diffphys-probe-" + str(os.getpid())
    images = {role: run(["docker", "image", "inspect", image, "--format", "{{.Id}}"]).strip()
              for role, image in [("ground", GROUND), ("onboard", ONBOARD)]}
    (report / "manifest.json").write_text(json.dumps(dict(images=images, domain=domain, port=port,
        network="host UDP with localhost only", worker="host CUDA", input="synthetic 36x48 positive depth",
        acceptance="20 sequential calls, each <100ms; exact reset/unit/endian equality; reject invalid input"), indent=2) + "\n")
    ground_ws = ROOT / "log/diffphys/ground_ws"
    ground_ws.mkdir(exist_ok=True)
    build = ["docker", "run", "--rm", "--network", "none", "--user", "{}:{}".format(os.getuid(), os.getgid()),
             "-e", "HOME=/tmp", "-v", str(ROOT) + ":/repo", "-w", "/repo/log/diffphys/ground_ws",
             "--entrypoint", "bash", GROUND, "-lc",
             "source /opt/ros/humble/setup.bash && colcon build --base-paths /repo/src/basic_function/inference/interfaces "
             "--packages-select uav_inference_interfaces --event-handlers console_direct+"]
    (report / "ground-build.txt").write_text(run(build))
    env = dict(os.environ, DIFFPHYS_DEVICE="cuda", INFERENCE_PORT=str(port))
    worker_log = (report / "worker.txt").open("w")
    worker = subprocess.Popen(["bash", str(ROOT / "deployment/inference/start_diffphys.sh")], env=env,
                              stdout=worker_log, stderr=subprocess.STDOUT, start_new_session=True)
    common = ["--init", "--network", "host", "-e", "ROS_DOMAIN_ID=" + str(domain), "-e", "ROS_LOCALHOST_ONLY=1",
              "-e", "FASTRTPS_DEFAULT_PROFILES_FILE=/repo/deployment/airsim/config/dds/fastdds_udp.xml",
              "-v", str(ROOT) + ":/repo:ro", "--entrypoint", "bash"]
    created = False
    try:
        opener = build_opener(ProxyHandler({}))
        deadline = time.monotonic() + 30
        while True:
            try:
                with opener.open("http://127.0.0.1:{}/health/ready".format(port), timeout=1) as response:
                    metadata = json.load(response)
                    assert metadata["device"] == "cuda"
                break
            except OSError:
                if worker.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Worker failed readiness; see worker.txt")
                time.sleep(0.1)
        (report / "ready.json").write_text(json.dumps(metadata, indent=2) + "\n")
        run(["docker", "run", "-d", "--name", name] + common + [ONBOARD, "-lc",
            "source /opt/ros/humble/setup.bash && source /repo/log/diffphys/ros_ws/install/setup.bash && "
            "exec python3 -m uav_inference.diffphys_node --ros-args -r __ns:=/diffphys_probe "
            "-p endpoint:=http://127.0.0.1:" + str(port)])
        created = True
        try:
            result = run(["docker", "run", "--rm"] + common + [GROUND, "-lc",
                "source /opt/ros/humble/setup.bash && source /repo/log/diffphys/ground_ws/install/setup.bash && "
                "python3 /repo/deployment/inference/tests/diffphys_remote_probe.py"])
        except subprocess.CalledProcessError as error:
            (report / "client-failure.txt").write_text(error.output)
            raise RuntimeError(error.output) from error
        (report / "result.json").write_text(result)
        print(result)
    finally:
        if created:
            (report / "adapter.txt").write_text(run(["docker", "logs", name]))
            run(["docker", "stop", "--timeout", "10", name], timeout=20)
            run(["docker", "rm", name])
        if worker.poll() is None:
            os.killpg(worker.pid, signal.SIGINT)
            try:
                worker.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(worker.pid, signal.SIGKILL)
                worker.wait(timeout=5)
        worker_log.close()
        (report / "cleanup.json").write_text(json.dumps(dict(worker_returncode=worker.returncode,
                                                             adapter_removed=created), indent=2) + "\n")
        print("Evidence:", report.relative_to(ROOT))


if __name__ == "__main__":
    main()
