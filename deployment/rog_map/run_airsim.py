#!/usr/bin/env python3
"""Own one visible fixed-image AirSim/PX4/AS2/ROG-Map acceptance run."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "deployment/airsim/demos/catora_open_field"))
sys.path.insert(0, str(ROOT / "simulator/airsim/sensors"))
from sim_startup import StartupRelay
from uav_as2_sensor_adapter.rpc import Rpc


PREFIX = "aerohub-rog-map-"
CONTAINERS = [PREFIX + name for name in (
    "clock", "vehicle", "sensors", "as2", "navigation", "map", "consumer")]
BOARD = "flight_board:v0.1.0"
ONBOARD = "onboard_computer:v0.1.0"
DOMAIN = 174
RPC_PORT = 44571
AIRSIM_TCP_PORT = 4990
RELAY_PORT = 5090
AGENT_PORT = 8972


def host_path(variable, default):
    value = Path(os.environ.get(variable, default))
    return (value if value.is_absolute() else ROOT / value).resolve()


ROG_WORKSPACE = host_path("ROG_MAP_WORKSPACE", "log/workspaces/rog_map")
SENSOR_WORKSPACE = host_path(
    "AIRSIM_SENSOR_WORKSPACE", "log/workspaces/airsim_sensors")
PX4_MSGS_WORKSPACE = host_path(
    "PX4_MSGS_OVERLAY_WORKSPACE", "log/workspaces/px4_msgs-rog-map-01")
AS2_RUN = ROOT / "deployment/airsim/as2/run.py"
ACCEPTANCE = ROOT / "deployment/rog_map/acceptance.py"
BASE_SETTINGS = ROOT / "simulator/airsim/scenarios/fast_livo2/settings.json"


def execute(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def container_exists(name):
    return subprocess.run(
        ["docker", "container", "inspect", name],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def manager(action, output):
    command = [
        sys.executable, ROOT / "simulator/airsim/manager/manager.py", action,
        "--profile", output / "airsim-profile.json",
        "--run-dir", output / "scene",
    ]
    binary = os.environ.get("AIRSIM_BINARY")
    if binary:
        command += ["--binary", Path(binary).expanduser().resolve()]
    return command


def write_runtime_configuration(output):
    settings = json.loads(BASE_SETTINGS.read_text())
    settings["ApiServerPort"] = RPC_PORT
    settings["Vehicles"]["PX4"]["TcpPort"] = AIRSIM_TCP_PORT
    settings["Vehicles"]["PX4"]["ControlPortLocal"] = 16840
    settings["Vehicles"]["PX4"]["ControlPortRemote"] = 16980
    runtime = output / "airsim-settings.json"
    runtime.write_text(json.dumps(settings, indent=2) + "\n")
    profile = {
        "binary_resource": "airsim_nh",
        "project": "AirSimNH",
        "settings": str(runtime),
        "args": [
            "-vulkan", "-nosound", "-NoMouseCapture",
            "-ini:Input:[/Script/Engine.InputSettings]:bCaptureMouseOnLaunch=False,"
            "DefaultViewportMouseCaptureMode=NoCapture,bDefaultViewportMouseLock=False,"
            "DefaultViewportMouseLockMode=DoNotLock",
        ],
    }
    (output / "airsim-profile.json").write_text(json.dumps(profile, indent=2) + "\n")

    px4_settings = json.loads(json.dumps(settings))
    px4_settings["Vehicles"]["PX4"]["TcpPort"] = RELAY_PORT
    (output / "px4-settings.json").write_text(json.dumps(px4_settings, indent=2) + "\n")
    (output / "fleet.yaml").write_text(yaml.safe_dump({"vehicles": {
        "drone0": {"fmu_prefix": "/drone0", "target_system_id": 72},
    }}))


def validate_inputs():
    required = (
        ROG_WORKSPACE / "install/setup.bash",
        ROG_WORKSPACE / "install/lib/uav_rog_map/rog_map_node",
        ROG_WORKSPACE / "install/lib/as2_navigation_inputs/navigation_inputs_node",
        ROG_WORKSPACE / "source-state.txt",
        SENSOR_WORKSPACE / "install/setup.bash",
        SENSOR_WORKSPACE / "install/lib/uav_as2_sensor_adapter/airsim_sensor_node",
        SENSOR_WORKSPACE / "source-state.txt",
        PX4_MSGS_WORKSPACE / "install/setup.bash",
        PX4_MSGS_WORKSPACE / "source-state.env",
        AS2_RUN,
        ACCEPTANCE,
        BASE_SETTINGS,
    )
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError("required acceptance inputs are missing: " + ", ".join(missing))
    for image in (BOARD, ONBOARD):
        execute(["docker", "image", "inspect", image], stdout=subprocess.DEVNULL)


def source_manifest(output, scenario):
    paths = [
        ROOT / "deployment/rog_map",
        ROOT / "src/perception/rog_map",
        ROOT / "src/perception/navigation_inputs",
        ROOT / "src/planner/super/ros2_adapter/native/rog_map",
        ROOT / "simulator/airsim/sensors",
        ROOT / "deployment/airsim/as2/run.py",
        BASE_SETTINGS,
    ]
    files = []
    for path in paths:
        files.extend(path.rglob("*") if path.is_dir() else [path])
    hashes = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files
        if path.is_file() and "__pycache__" not in path.parts
    }
    manifest = {
        "repository_commit": subprocess.check_output(
            ["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip(),
        "repository_status": subprocess.check_output(
            ["git", "-C", ROOT, "status", "--short"], text=True).splitlines(),
        "scenario": scenario,
        "ros_domain": DOMAIN,
        "render": {
            "display": os.environ.get("DISPLAY"),
            "mode": os.environ.get("AIRSIM_RENDER_MODE"),
        },
        "ports": {
            "airsim_rpc": RPC_PORT,
            "airsim_tcp": AIRSIM_TCP_PORT,
            "px4_relay": RELAY_PORT,
            "xrce_agent": AGENT_PORT,
        },
        "images": {
            image: subprocess.check_output(
                ["docker", "image", "inspect", image, "--format", "{{.Id}}"],
                text=True).strip()
            for image in (BOARD, ONBOARD)
        },
        "workspaces": {
            "rog_map": {
                "path": str(ROG_WORKSPACE),
                "source_state_sha256": hashlib.sha256(
                    (ROG_WORKSPACE / "source-state.txt").read_bytes()).hexdigest(),
            },
            "airsim_sensors": {
                "path": str(SENSOR_WORKSPACE),
                "source_state_sha256": hashlib.sha256(
                    (SENSOR_WORKSPACE / "source-state.txt").read_bytes()).hexdigest(),
            },
            "px4_msgs_diagnostics": {
                "path": str(PX4_MSGS_WORKSPACE),
                "source_state_sha256": hashlib.sha256(
                    (PX4_MSGS_WORKSPACE / "source-state.env").read_bytes()).hexdigest(),
            },
        },
        "source_hashes": hashes,
    }
    (output / "source-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (output / "images.json").write_text(json.dumps(manifest["images"], indent=2) + "\n")


def run_detached(name, image, environment, volumes, command, labels=None):
    args = [
        "docker", "run", "--detach", "--init", "--network", "host", "--name", name,
    ]
    for key, value in (labels or {}).items():
        args += ["--label", f"{key}={value}"]
    for key, value in environment.items():
        args += ["--env", f"{key}={value}"]
    for source, target, mode in volumes:
        args += ["--volume", f"{source}:{target}:{mode}"]
    args += ["--entrypoint", "bash", image, "-lc", command]
    execute(args, stdout=subprocess.DEVNULL)


def start_map():
    run_detached(
        PREFIX + "map", ONBOARD, {"ROS_DOMAIN_ID": str(DOMAIN)},
        [(ROG_WORKSPACE, "/workspace", "ro")],
        "source /opt/ros/humble/setup.bash && "
        "source /root/aerostack2_ws/install/setup.bash && "
        "source /workspace/install/setup.bash && "
        "exec ros2 launch uav_rog_map rog_map.launch.py "
        "namespace:=drone0 global_frame:=drone0/odom sensor_frame:=drone0/Lidar",
        {"aerohub.task": "rog-map-runtime", "aerohub.namespace": "drone0"})


def wait_px4(output):
    started = time.monotonic()
    deadline = started + 90
    while time.monotonic() < deadline:
        log = subprocess.check_output(
            ["docker", "logs", PREFIX + "vehicle"], stderr=subprocess.STDOUT, text=True)
        if "Ready for takeoff!" in log:
            evidence = {
                "ready": True,
                "startup_complete": "Startup script returned successfully" in log,
                "startup_wall_seconds": time.monotonic() - started,
                "image_embedded_px4": True,
                "relay_delay_s": 0.1,
            }
            (output / "px4-startup.json").write_text(json.dumps(evidence, indent=2) + "\n")
            return
        time.sleep(0.25)
    raise RuntimeError("PX4 did not reach its startup-ready log state")


def screenshot(output, name):
    rpc = Rpc("127.0.0.1", RPC_PORT, timeout=5)
    try:
        request = {
            "camera_name": "survey", "image_type": 0,
            "pixels_as_float": False, "compress": True,
        }
        response = rpc.call("simGetImages", [request], "", True)[0]
        payload = bytes(response["image_data_uint8"])
        if len(payload) < 1024:
            raise RuntimeError("AirSim screenshot payload is unexpectedly small")
        (output / (name + ".png")).write_bytes(payload)
    finally:
        rpc.close()


def wait_file(path, timeout=180):
    deadline = time.monotonic() + timeout
    while not path.is_file():
        if time.monotonic() >= deadline:
            raise TimeoutError("timed out waiting for " + str(path))
        time.sleep(0.1)


def save_container_evidence(name, output, suffix=""):
    if not container_exists(name):
        return
    stem = name + suffix
    with (output / (stem + ".inspect.json")).open("w") as stream:
        subprocess.run(["docker", "inspect", name], stdout=stream, stderr=subprocess.STDOUT)
    with (output / (stem + ".log")).open("w") as stream:
        subprocess.run(["docker", "logs", name], stdout=stream, stderr=subprocess.STDOUT)


def inject_fault(scenario, output, errors):
    try:
        if scenario == "sensor_fault":
            wait_file(output / "sensor-fault-ready.json")
            target = PREFIX + "sensors"
            started = time.time()
            execute(["docker", "pause", target], stdout=subprocess.DEVNULL)
            (output / "sensor-fault-injected.json").write_text(json.dumps({
                "target_container": target,
                "operation": "docker pause",
                "wall_time": started,
            }, indent=2) + "\n")
            time.sleep(2.25)
            execute(["docker", "unpause", target], stdout=subprocess.DEVNULL)
            (output / "sensor-fault-recovered.json").write_text(json.dumps({
                "target_container": target,
                "operation": "docker unpause",
                "wall_time": time.time(),
            }, indent=2) + "\n")
            return

        if scenario != "algorithm_fault":
            return
        wait_file(output / "algorithm-fault-ready.json")
        target = PREFIX + "map"
        save_container_evidence(target, output, "-before-fault")
        started = time.time()
        execute(["docker", "stop", "--timeout", "1", target], stdout=subprocess.DEVNULL)
        execute(["docker", "rm", target], stdout=subprocess.DEVNULL)
        (output / "algorithm-fault-injected.json").write_text(json.dumps({
            "target_container": target,
            "operation": "docker stop",
            "wall_time": started,
        }, indent=2) + "\n")
        wait_file(output / "algorithm-unavailable-observed.json", 30)
        sensor = PREFIX + "sensors"
        execute(["docker", "pause", sensor], stdout=subprocess.DEVNULL)
        start_map()
        (output / "algorithm-restarted.json").write_text(json.dumps({
            "target_container": target,
            "sensor_container_paused": sensor,
            "wall_time": time.time(),
        }, indent=2) + "\n")
        wait_file(output / "algorithm-empty-observed.json", 30)
        execute(["docker", "unpause", sensor], stdout=subprocess.DEVNULL)
        (output / "algorithm-input-resumed.json").write_text(json.dumps({
            "target_container": sensor,
            "operation": "docker unpause",
            "wall_time": time.time(),
        }, indent=2) + "\n")
    except BaseException as error:
        errors.append(repr(error))


def cleanup(output):
    for name in reversed(CONTAINERS):
        if not container_exists(name):
            continue
        subprocess.run(["docker", "unpause", name], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        save_container_evidence(name, output)
        subprocess.run(
            ["docker", "stop", "--timeout", "10", name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(
            ["docker", "rm", "--force", name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(manager("stop", output), check=False, stdout=subprocess.DEVNULL)


def port_available(port):
    try:
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False


def write_cleanup_evidence(output):
    evidence = {
        "containers_remaining": [name for name in CONTAINERS if container_exists(name)],
        "ports_available": {
            str(port): port_available(port)
            for port in (RPC_PORT, AIRSIM_TCP_PORT, RELAY_PORT, AGENT_PORT)
        },
        "wall_time": time.time(),
    }
    evidence["success"] = (
        not evidence["containers_remaining"] and all(evidence["ports_available"].values()))
    (output / "cleanup.json").write_text(json.dumps(evidence, indent=2) + "\n")


def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if os.environ.get("DISPLAY") != ":3" or os.environ.get("AIRSIM_RENDER_MODE") != "windowed":
        raise RuntimeError("acceptance requires DISPLAY=:3 and AIRSIM_RENDER_MODE=windowed")
    if any((output / name).exists() for name in ("result.json", "failure.json", "images.json")):
        raise RuntimeError("choose a fresh output directory")
    existing = [name for name in CONTAINERS if container_exists(name)]
    if existing:
        raise RuntimeError("reserved acceptance containers already exist: " + ", ".join(existing))

    validate_inputs()
    write_runtime_configuration(output)
    source_manifest(output, args.scenario)
    relay = None
    injector = None
    injector_errors = []
    succeeded = False
    try:
        execute(manager("start", output) + ["--timeout", "90"])
        state = json.loads((output / "scene/state.json").read_text())
        pid = state["pid"]
        environment = Path(f"/proc/{pid}/environ").read_bytes().replace(
            b"\0", b"\n").decode(errors="replace")
        window = subprocess.check_output(
            ["bash", "-lc", "DISPLAY=:3 xwininfo -root -tree | grep -i AirSimNH"],
            text=True)
        (output / "window-evidence.txt").write_text(
            "command=" + json.dumps(state["command"]) + "\n" + environment + window)
        relay = StartupRelay(
            ("127.0.0.1", RELAY_PORT), ("127.0.0.1", AIRSIM_TCP_PORT), delay=0.1)

        execute([
            "docker", "run", "--detach", "--init", "--network", "host",
            "--name", PREFIX + "clock", "--env", f"ROS_DOMAIN_ID={DOMAIN}",
            "--env", "VEHICLE_NAME=PX4", "--env", f"AIRSIM_RPC_PORT={RPC_PORT}",
            BOARD, "clock",
        ], stdout=subprocess.DEVNULL)
        vehicle = [
            "docker", "run", "--detach", "--init", "--network", "host",
            "--name", PREFIX + "vehicle",
        ]
        for key, value in {
                "ROS_DOMAIN_ID": DOMAIN,
                "VEHICLE_NAME": "PX4",
                "VEHICLE_NAMESPACE": "drone0",
                "PX4_INSTANCE": 71,
                "PX4_SYSTEM_ID": 72,
                "XRCE_AGENT_PORT": AGENT_PORT,
                "AIRSIM_RPC_PORT": RPC_PORT,
                "FLIGHT_BOARD_SENSORS": 0,
                "PX4_PARAM_COM_ARM_WO_GPS": 1,
                "PX4_PARAM_CBRK_SUPPLY_CHK": 894281,
        }.items():
            vehicle += ["--env", f"{key}={value}"]
        vehicle += [
            "--volume", f"{output / 'px4-settings.json'}:/config/settings.json:ro",
            BOARD, "vehicle",
        ]
        execute(vehicle, stdout=subprocess.DEVNULL)
        wait_px4(output)

        sensor_command = (
            "source /opt/ros/humble/setup.bash && "
            "source /opt/platform_sensors/install/setup.bash && "
            "source /sensor-overlay/install/setup.bash && "
            "exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args "
            "-p use_sim_time:=true -p settings_file:=/config/settings.json "
            "-p vehicle_name:=PX4 -p vehicle_namespace:=drone0 "
            f"-p rpc_host:=127.0.0.1 -p rpc_port:={RPC_PORT} "
            "-p camera_rate_hz:=5.0 -p imu_rate_hz:=100.0 -p lidar_rate_hz:=10.0")
        run_detached(
            PREFIX + "sensors", BOARD, {"ROS_DOMAIN_ID": str(DOMAIN)},
            [(SENSOR_WORKSPACE, "/sensor-overlay", "ro"),
             (output / "airsim-settings.json", "/config/settings.json", "ro")],
            sensor_command, {"aerohub.task": "rog-map-sensors"})

        as2_command = (
            "source /opt/ros/humble/setup.bash && "
            "source /opt/px4_ws/install/setup.bash && "
            "source /px4-overlay/install/setup.bash && "
            "source /root/aerostack2_ws/install/setup.bash && "
            "exec python3 /opt/aerohub/run_as2.py")
        run_detached(
            PREFIX + "as2", ONBOARD, {
                "ROS_DOMAIN_ID": str(DOMAIN),
                "AS2_FLEET_FILE": "/results/fleet.yaml",
                "AS2_EXTERNAL_ODOM": "0",
            },
            [(output, "/results", "rw"),
             (PX4_MSGS_WORKSPACE, "/px4-overlay", "ro"),
             (AS2_RUN, "/opt/aerohub/run_as2.py", "ro")],
            as2_command, {"aerohub.task": "rog-map-as2"})

        navigation_command = (
            "source /opt/ros/humble/setup.bash && "
            "source /root/aerostack2_ws/install/setup.bash && "
            "source /workspace/install/setup.bash && "
            "exec ros2 run as2_navigation_inputs navigation_inputs_node --ros-args "
            "-r __ns:=/drone0 --params-file "
            "/workspace/install/share/as2_navigation_inputs/config/drone0.yaml")
        run_detached(
            PREFIX + "navigation", ONBOARD, {"ROS_DOMAIN_ID": str(DOMAIN)},
            [(ROG_WORKSPACE, "/workspace", "ro")], navigation_command,
            {"aerohub.task": "rog-map-navigation-inputs"})
        start_map()
        screenshot(output, "initialized")

        if args.scenario != "normal":
            injector = threading.Thread(
                target=inject_fault,
                args=(args.scenario, output, injector_errors), daemon=True)
            injector.start()

        consumer_command = (
            "source /opt/ros/humble/setup.bash && "
            "source /opt/px4_ws/install/setup.bash && "
            "source /px4-overlay/install/setup.bash && "
            "source /root/aerostack2_ws/install/setup.bash && "
            "source /workspace/install/setup.bash && "
            "exec python3 /opt/aerohub/acceptance.py")
        consumer = [
            "docker", "run", "--init", "--network", "host",
            "--name", PREFIX + "consumer",
            "--env", f"ROS_DOMAIN_ID={DOMAIN}",
            "--env", f"ROG_MAP_SCENARIO={args.scenario}",
            "--env", "ROG_MAP_RESULTS=/results",
            "--volume", f"{output}:/results:rw",
            "--volume", f"{ROG_WORKSPACE}:/workspace:ro",
            "--volume", f"{PX4_MSGS_WORKSPACE}:/px4-overlay:ro",
            "--volume", f"{ACCEPTANCE}:/opt/aerohub/acceptance.py:ro",
            "--entrypoint", "bash", ONBOARD, "-lc", consumer_command,
        ]
        with (output / "consumer-console.log").open("w") as stream:
            execute(consumer, stdout=stream, stderr=subprocess.STDOUT)
        if injector:
            injector.join(timeout=20)
            if injector.is_alive():
                raise RuntimeError("fault injector did not terminate")
            if injector_errors:
                raise RuntimeError("; ".join(injector_errors))
        result = json.loads((output / "result.json").read_text())
        if not result.get("success"):
            raise RuntimeError("ROG-Map acceptance consumer reported failure")
        screenshot(output, "completed")
        succeeded = True
    finally:
        if relay:
            relay.close()
        cleanup(output)
        write_cleanup_evidence(output)
        if succeeded:
            cleanup_result = json.loads((output / "cleanup.json").read_text())
            if not cleanup_result["success"]:
                raise RuntimeError("acceptance assertions passed but cleanup failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("run", "stop", "status"))
    parser.add_argument(
        "--scenario", choices=("normal", "sensor_fault", "algorithm_fault"),
        default="normal")
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "log/airsim/rog-map")
    args = parser.parse_args()
    output = args.output.resolve()
    if args.action == "stop":
        output.mkdir(parents=True, exist_ok=True)
        cleanup(output)
        write_cleanup_evidence(output)
        return
    if args.action == "status":
        subprocess.run(manager("status", output), check=False)
        execute(["docker", "ps", "--filter", "name=" + PREFIX])
        return
    run(args)


if __name__ == "__main__":
    main()
