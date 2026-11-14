#!/usr/bin/env python3
"""Own one visible AirSim/PX4/PIN-SLAM/AS2 acceptance session."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import yaml


ROOT = Path(__file__).resolve().parents[2]
PREFIX = "aerohub-pin-slam-"
CONTAINERS = [PREFIX + name for name in (
    "clock", "vehicle", "sensors", "truth", "algorithm", "as2",
    "freshness", "mission")]
BOARD = "flight_board:v0.1.0"
ONBOARD = "onboard_computer:v0.1.0"
DOMAIN = 81


def host_path(variable, default):
    value = Path(os.environ.get(variable, default))
    return (value if value.is_absolute() else ROOT / value).resolve()


PIN_WORKSPACE = host_path("PIN_SLAM_WORKSPACE", "log/workspaces/pin_slam")
SENSOR_WORKSPACE = host_path(
    "AIRSIM_SENSOR_WORKSPACE", "log/workspaces/airsim-sensors-dpv-parallel-rpc-02")
PX4_WORKSPACE = host_path(
    "PX4_MSGS_OVERLAY_WORKSPACE", "log/workspaces/px4_msgs-61907962-script-validation-02")
AS2_WORKSPACE = host_path(
    "AS2_EXECUTION_WORKSPACE", "log/workspaces/as2-execution-fast-livo2-ca38fa8-01")
SETTINGS = ROOT / "deployment/pin_slam/airsim-settings.json"
PROFILE = ROOT / "deployment/pin_slam/airsim-profile.json"
AS2_RUN = ROOT / "deployment/airsim/as2/run.py"
VALIDATOR = ROOT / "deployment/pin_slam/shadow_validation.py"
TRUTH = ROOT / "deployment/pin_slam/airsim_truth_bridge.py"
FRESHNESS = ROOT / "deployment/state_estimator/canonical_freshness.py"
GATE = ROOT / "deployment/pin_slam/gate_result.py"
GATE_CONFIG = ROOT / "deployment/pin_slam/validation.yaml"
ESTIMATOR_CONFIG = ROOT / "src/state_estimator/pin_slam/plugin/config/estimator.yaml"
ALGORITHM_CONFIG = ROOT / "deployment/pin_slam/airsim.yaml"
UPSTREAM_ROOT = ROOT / "src/state_estimator/pin_slam/upstream"


def execute(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def manager(action, output):
    return [sys.executable, ROOT / "simulator/airsim/manager/manager.py", action,
            "--profile", PROFILE, "--run-dir", output / "scene"]


def container_exists(name):
    return subprocess.run(
        ["docker", "container", "inspect", name], stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL).returncode == 0


def run_detached(name, image, environment, volumes, command, gpu=False):
    args = ["docker", "run", "--detach", "--init", "--network", "host",
            "--name", name]
    if gpu:
        args += ["--runtime", "nvidia", "--env", "NVIDIA_VISIBLE_DEVICES=0",
                 "--env", "NVIDIA_DRIVER_CAPABILITIES=compute,utility"]
    for key, value in environment.items():
        args += ["--env", f"{key}={value}"]
    for source, target, mode in volumes:
        args += ["--volume", f"{source}:{target}:{mode}"]
    args += ["--entrypoint", "bash", image, "-lc", command]
    execute(args, stdout=subprocess.DEVNULL)


def cleanup(output):
    for name in reversed(CONTAINERS):
        if not container_exists(name):
            continue
        subprocess.run(["docker", "stop", "--timeout", "10", name],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with (output / f"{name}.log").open("w") as stream:
            subprocess.run(["docker", "logs", name], stdout=stream,
                           stderr=subprocess.STDOUT)
        with (output / f"{name}.inspect.json").open("w") as stream:
            subprocess.run(["docker", "inspect", name], stdout=stream,
                           stderr=subprocess.STDOUT)
        subprocess.run(["docker", "rm", "--force", name],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(manager("stop", output), check=False, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)


def workspace_state(workspace):
    for name in ("source-state.txt", "source-state.env"):
        candidate = workspace / name
        if candidate.is_file():
            return candidate
    raise RuntimeError(f"workspace source manifest is missing: {workspace}")


def validate_inputs():
    required = {
        "pin_ros": PIN_WORKSPACE / "ros/install/setup.bash",
        "pin_node": PIN_WORKSPACE / "ros/install/lib/uav_pin_slam_ros2/pin_slam_node",
        "pin_python": PIN_WORKSPACE / "venv/bin/python",
        "pin_config": ALGORITHM_CONFIG,
        "pin_upstream": UPSTREAM_ROOT / "utils/config.py",
        "sensor": SENSOR_WORKSPACE / "install/lib/uav_as2_sensor_adapter/airsim_sensor_node",
        "px4_msgs": PX4_WORKSPACE / "install/setup.bash",
        "as2": AS2_WORKSPACE / "install/lib/as2_platform_pixhawk/as2_platform_pixhawk_node",
    }
    for label, path in required.items():
        if not path.exists():
            raise RuntimeError(f"{label} artifact is missing: {path}")
    for workspace in (PIN_WORKSPACE, SENSOR_WORKSPACE, PX4_WORKSPACE, AS2_WORKSPACE):
        workspace_state(workspace)


def source_manifest(output, scenario):
    paths = [ROOT / "deployment/pin_slam", ROOT / "deployment/flight_board",
             ROOT / "deployment/airsim/as2", ROOT / "src/state_estimator/pin_slam",
             ROOT / "simulator/airsim/sensors"]
    files = []
    for path in paths:
        files.extend(item for item in path.rglob("*") if item.is_file())
    hashes = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files if "__pycache__" not in path.parts
    }
    workspaces = {}
    for label, workspace in {
            "pin_slam": PIN_WORKSPACE, "airsim_sensors": SENSOR_WORKSPACE,
            "px4_msgs": PX4_WORKSPACE, "as2_execution": AS2_WORKSPACE}.items():
        state = workspace_state(workspace)
        workspaces[label] = {
            "path": str(workspace),
            "source_state_sha256": hashlib.sha256(state.read_bytes()).hexdigest(),
            "source_state": state.read_text().splitlines(),
        }
    manifest = {
        "scenario": scenario,
        "commit": subprocess.check_output(
            ["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip(),
        "pin_slam_commit": subprocess.check_output(
            ["git", "-C", UPSTREAM_ROOT,
             "rev-parse", "HEAD"], text=True).strip(),
        "dirty": bool(subprocess.check_output(
            ["git", "-C", ROOT, "status", "--porcelain"])),
        "render": {"display": os.environ.get("DISPLAY"),
                   "mode": os.environ.get("AIRSIM_RENDER_MODE")},
        "ros_domain": DOMAIN,
        "ports": {"airsim_rpc": 41471, "px4_tcp": 4580,
                  "agent": 8891, "system_id": 21},
        "workspaces": workspaces,
        "source_hashes": hashes,
    }
    (output / "source-manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n")
    with (output / "dirty.patch").open("w") as stream:
        subprocess.run(["git", "-C", ROOT, "diff", "--binary"], stdout=stream,
                       check=True)


def wait_px4(output):
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        log = subprocess.check_output(
            ["docker", "logs", PREFIX + "vehicle"], stderr=subprocess.STDOUT,
            text=True)
        if "Ready for takeoff!" in log:
            (output / "px4-startup.json").write_text(json.dumps({
                "ready": True,
                "firmware_commit": "61907962c9220437ec01a98fa26bfbf5cbb254b5",
                "source": "flight_board:v0.1.0",
            }, indent=2) + "\n")
            return
        time.sleep(0.25)
    raise RuntimeError("PX4 did not become ready")


def graph_evidence(output):
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source /opt/px4_ws/install/setup.bash && source /px4-overlay/install/setup.bash && "
        "source /root/aerostack2_ws/install/setup.bash && "
        "source /as2-overlay/install/setup.bash && source /pin/ros/install/setup.bash && "
        "ros2 node list; ros2 topic info -v "
        "/drone0/sensor_measurements/Lidar/points; "
        "ros2 topic info -v /pin_slam/validation/airsim_truth; "
        "ros2 topic info -v /drone0/pin_slam/odometry; "
        "ros2 topic info -v /drone0/self_localization/pose")
    base = [
        "docker", "run", "--rm", "--network", "host",
        "--env", f"ROS_DOMAIN_ID={DOMAIN}",
        "--volume", f"{PX4_WORKSPACE}:/px4-overlay:ro",
        "--volume", f"{AS2_WORKSPACE}:/as2-overlay:ro",
        "--volume", f"{PIN_WORKSPACE}:/pin:ro",
        "--entrypoint", "bash", ONBOARD, "-lc", command,
    ]
    deadline = time.monotonic() + 60
    attempts = []
    while True:
        completed = subprocess.run(base, text=True, capture_output=True)
        text = completed.stdout + "\nSTDERR:\n" + completed.stderr
        attempts.append(text)
        ready = (
            completed.returncode == 0
            and "/drone0/pin_slam" in completed.stdout
            and "pin_slam_airsim_truth_bridge" in completed.stdout
            and completed.stdout.count("Node name: pin_slam") >= 2
            and "Unknown topic" not in completed.stderr
        )
        if ready:
            (output / "ros-graph.txt").write_text(text)
            (output / "ros-graph-attempts.txt").write_text(
                "\n\n===== ATTEMPT =====\n".join(attempts))
            return
        if time.monotonic() >= deadline:
            (output / "ros-graph-attempts.txt").write_text(
                "\n\n===== ATTEMPT =====\n".join(attempts))
            raise RuntimeError("ROS graph did not become complete")
        time.sleep(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("run", "stop", "status"))
    parser.add_argument("--scenario", choices=(
        "shadow", "authority", "sensor_fault", "algorithm_fault"),
        default="shadow")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "log/pin_slam/airsim-acceptance")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.action == "stop":
        cleanup(output)
        return
    if args.action == "status":
        subprocess.run(manager("status", output), check=False)
        execute(["docker", "ps", "--filter", "name=" + PREFIX])
        return
    if os.environ.get("DISPLAY") != ":3" or os.environ.get(
            "AIRSIM_RENDER_MODE") != "windowed":
        raise RuntimeError("acceptance requires DISPLAY=:3 and windowed rendering")
    if any((output / name).exists() for name in (
            "result.json", "failure.json", "gate-report.json")):
        raise RuntimeError("choose a fresh output directory")
    if any(container_exists(name) for name in CONTAINERS):
        raise RuntimeError("a reserved PIN-SLAM container already exists")
    for image in (BOARD, ONBOARD):
        execute(["docker", "image", "inspect", image], stdout=subprocess.DEVNULL)
    validate_inputs()
    source_manifest(output, args.scenario)
    (output / "fleet.yaml").write_text(yaml.safe_dump({"vehicles": {
        "drone0": {"fmu_prefix": "/drone0", "target_system_id": 21}}}))
    images = {
        label: subprocess.check_output(
            ["docker", "image", "inspect", "--format", "{{.Id}}", image],
            text=True).strip()
        for label, image in (("flight_board", BOARD), ("onboard_computer", ONBOARD))
    }
    (output / "images.json").write_text(json.dumps(images, indent=2) + "\n")
    try:
        execute(manager("start", output) + ["--timeout", "90"])
        scene = json.loads((output / "scene/state.json").read_text())
        environment = Path(f"/proc/{scene['pid']}/environ").read_bytes().replace(
            b"\0", b"\n").decode(errors="replace")
        window = subprocess.check_output(
            ["bash", "-lc", "DISPLAY=:3 xwininfo -root -tree | grep -i AirSimNH"],
            text=True)
        (output / "window-evidence.txt").write_text(
            "command=" + json.dumps(scene["command"]) + "\n" + environment + window)

        common = {"ROS_DOMAIN_ID": str(DOMAIN)}
        clock_env = dict(common, AIRSIM_RPC_PORT="41471", VEHICLE_NAME="PX4")
        run_detached(PREFIX + "clock", BOARD, clock_env, [],
                     "source /opt/ros/humble/setup.bash && "
                     "source /opt/platform_sensors/install/setup.bash && "
                     "exec python3 /opt/flight_board/supervisor.py clock")
        vehicle_env = dict(common, VEHICLE_NAME="PX4", VEHICLE_NAMESPACE="drone0",
                           PX4_INSTANCE="20", PX4_SYSTEM_ID="21",
                           XRCE_AGENT_PORT="8891", AIRSIM_RPC_PORT="41471",
                           AIRSIM_SETTINGS="/config/settings.json",
                           FLIGHT_BOARD_SENSORS="0")
        run_detached(PREFIX + "vehicle", BOARD, vehicle_env,
                     [(SETTINGS, "/config/settings.json", "ro")],
                     "source /opt/ros/humble/setup.bash && "
                     "source /opt/platform_sensors/install/setup.bash && "
                     "exec python3 /opt/flight_board/supervisor.py vehicle")
        wait_px4(output)

        sensor_command = (
            "source /opt/ros/humble/setup.bash && source /opt/platform_sensors/install/setup.bash && "
            "source /sensor-overlay/install/setup.bash && "
            "exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args "
            "-p use_sim_time:=true -p settings_file:=/config/settings.json "
            "-p vehicle_name:=PX4 -p vehicle_namespace:=drone0 "
            "-p rpc_port:=41471 -p camera_enabled:=false "
            "-p imu_rate_hz:=100.0 -p lidar_rate_hz:=4.0")
        run_detached(PREFIX + "sensors", BOARD, common,
                     [(SENSOR_WORKSPACE, "/sensor-overlay", "ro"),
                      (SETTINGS, "/config/settings.json", "ro")], sensor_command)
        truth_command = (
            "source /opt/ros/humble/setup.bash && source /sensor-overlay/install/setup.bash && "
            "exec python3 /opt/aerohub/airsim_truth_bridge.py --ros-args "
            "-p use_sim_time:=true -p rpc_port:=41471")
        run_detached(PREFIX + "truth", BOARD, common,
                     [(SENSOR_WORKSPACE, "/sensor-overlay", "ro"),
                      (TRUTH, "/opt/aerohub/airsim_truth_bridge.py", "ro")],
                     truth_command)

        algorithm_command = (
            "source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && "
            "source /pin/venv/bin/activate && source /pin/ros/install/setup.bash && "
            "exec /pin/venv/bin/python /pin/ros/install/lib/uav_pin_slam_ros2/pin_slam_node --ros-args "
            "-r __ns:=/drone0 --params-file /config/algorithm.yaml")
        algorithm_env = dict(
            common,
            PIN_SLAM_ROOT="/repo/src/state_estimator/pin_slam/upstream",
            PYTHONPATH="/repo/src/state_estimator/pin_slam/upstream",
        )
        run_detached(PREFIX + "algorithm", ONBOARD, algorithm_env,
                     [(ROOT, "/repo", "ro"), (PIN_WORKSPACE, "/pin", "ro"),
                      (ALGORITHM_CONFIG, "/config/airsim.yaml", "ro"),
                      (ROOT / "deployment/pin_slam/algorithm.params.yaml",
                       "/config/algorithm.yaml", "ro"), (output, "/results", "rw")],
                     algorithm_command, gpu=True)

        as2_env = dict(common, AS2_FLEET_FILE="/results/fleet.yaml",
                       AS2_EXTERNAL_ODOM="0")
        if args.scenario != "shadow":
            as2_env.update({
                "AS2_STATE_ESTIMATOR_PLUGIN": "pin_slam",
                "AS2_STATE_ESTIMATOR_CONFIG_FILE": "/config/estimator.yaml",
                "AS2_STATE_ESTIMATOR_LAUNCH_PACKAGE": "uav_pin_slam",
                "AS2_STATE_ESTIMATOR_LAUNCH_FILE": "pin_slam.launch.py",
            })
        as2_command = (
            "source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && "
            "source /px4-overlay/install/setup.bash && "
            "source /root/aerostack2_ws/install/setup.bash && "
            "source /as2-overlay/install/setup.bash && source /pin/ros/install/setup.bash && "
            "exec python3 /opt/aerohub/run_as2.py")
        run_detached(PREFIX + "as2", ONBOARD, as2_env,
                     [(output, "/results", "rw"), (PX4_WORKSPACE, "/px4-overlay", "ro"),
                      (AS2_WORKSPACE, "/as2-overlay", "ro"),
                      (PIN_WORKSPACE, "/pin", "ro"),
                      (AS2_RUN, "/opt/aerohub/run_as2.py", "ro"),
                      (ESTIMATOR_CONFIG, "/config/estimator.yaml", "ro")], as2_command)

        if args.scenario != "shadow":
            freshness_command = (
                "source /opt/ros/humble/setup.bash && "
                "source /root/aerostack2_ws/install/setup.bash && "
                "exec python3 /probe.py --namespace drone0 "
                "--output /results/canonical-freshness.json")
            run_detached(PREFIX + "freshness", ONBOARD, common,
                         [(output, "/results", "rw"), (FRESHNESS, "/probe.py", "ro")],
                         freshness_command)

        time.sleep(8)
        graph_evidence(output)
        mission_environment = dict(common, PIN_SLAM_SHADOW_RESULTS="/results",
                                   PIN_SLAM_SCENARIO=args.scenario)
        mission_command = (
            "source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && "
            "source /px4-overlay/install/setup.bash && "
            "source /root/aerostack2_ws/install/setup.bash && "
            "source /as2-overlay/install/setup.bash && exec python3 /validator.py")
        mission_args = ["docker", "run", "--init", "--network", "host",
                        "--name", PREFIX + "mission"]
        for key, value in mission_environment.items():
            mission_args += ["--env", f"{key}={value}"]
        for source, target, mode in (
                (output, "/results", "rw"), (PX4_WORKSPACE, "/px4-overlay", "ro"),
                (AS2_WORKSPACE, "/as2-overlay", "ro"),
                (VALIDATOR, "/validator.py", "ro")):
            mission_args += ["--volume", f"{source}:{target}:{mode}"]
        mission_args += ["--entrypoint", "bash", ONBOARD, "-lc", mission_command]

        injector_errors = []
        injector = None
        if args.scenario in ("sensor_fault", "algorithm_fault"):
            target = PREFIX + ("sensors" if args.scenario == "sensor_fault" else "algorithm")

            def inject_fault_inner():
                marker = output / "fault-ready.json"
                deadline = time.monotonic() + 180
                while not marker.is_file() and time.monotonic() < deadline:
                    time.sleep(0.1)
                if not marker.is_file():
                    injector_errors.append("fault-ready marker timeout")
                    return
                completed = subprocess.run(
                    ["docker", "stop", "--timeout", "1", target],
                    text=True, capture_output=True)
                (output / "fault-injection.json").write_text(json.dumps({
                    "scenario": args.scenario, "target_container": target,
                    "returncode": completed.returncode, "stdout": completed.stdout,
                    "stderr": completed.stderr, "wall_time": time.time(),
                }, indent=2) + "\n")
                if completed.returncode:
                    injector_errors.append(
                        f"docker stop failed for {target}: "
                        f"returncode={completed.returncode}, "
                        f"stderr={completed.stderr.strip()!r}")

            def inject_fault():
                try:
                    inject_fault_inner()
                except BaseException as error:
                    injector_errors.append(repr(error))
                    (output / "fault-injection-error.json").write_text(json.dumps({
                        "scenario": args.scenario,
                        "target_container": target,
                        "error": repr(error),
                        "wall_time": time.time(),
                    }, indent=2) + "\n")

            injector = threading.Thread(target=inject_fault, daemon=True)
            injector.start()
        with (output / "mission-console.log").open("w") as stream:
            execute(mission_args, stdout=stream, stderr=subprocess.STDOUT)
        if injector:
            injector.join(10)
            if injector.is_alive():
                injector_errors.append(
                    "fault injector did not finish within 10 seconds after mission exit")
            if injector_errors:
                raise RuntimeError("fault injector failed: " + "; ".join(injector_errors))
        if container_exists(PREFIX + "freshness"):
            execute(["docker", "stop", "--timeout", "10", PREFIX + "freshness"],
                    stdout=subprocess.DEVNULL)
        gate_command = [sys.executable, GATE, GATE_CONFIG, output / "result.json",
                        "--output", output / "gate-report.json"]
        if args.scenario == "authority":
            gate_command += ["--authority", "--authority-freshness",
                             output / "canonical-freshness.json"]
        if args.scenario in ("shadow", "authority"):
            execute(gate_command)
    finally:
        cleanup(output)


if __name__ == "__main__":
    main()
