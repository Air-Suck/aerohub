#!/usr/bin/env python3
"""Ground-station Docker -> native AS2 in onboard Docker, without flight."""

import hashlib
import json
import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
GROUND_IMAGE = "osrf/ros:humble-desktop-full-jammy"
ONBOARD_IMAGE = "onboard_computer:v0.1.0"


def run(arguments, timeout=120):
    return subprocess.check_output(arguments, text=True, stderr=subprocess.STDOUT, timeout=timeout)


def main():
    output = ROOT / "log/inference/as2-baseline" / str(os.getpid())
    output.mkdir(parents=True)
    package = output / "ws/src/as2_msgs"
    (package / "srv").mkdir(parents=True)
    # Compile only the canonical query type on the ground station; no native
    # AS2 binaries/libraries are copied to the workstation.
    definition = "/root/aerostack2_ws/install/share/as2_msgs/srv/ListControlModes.srv"
    service = run(["docker", "run", "--rm", "--network", "none", "--entrypoint", "cat", ONBOARD_IMAGE, definition])
    (package / "srv/ListControlModes.srv").write_text(service)
    (package / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.8)
project(as2_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/ListControlModes.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
''')
    (package / "package.xml").write_text('''<package format="3">
<name>as2_msgs</name><version>0.0.0</version><description>Canonical AS2 query interface subset for baseline client</description>
<maintainer email="noah@example.com">noah</maintainer><license>BSD-3-Clause</license>
<buildtool_depend>ament_cmake</buildtool_depend><build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend><member_of_group>rosidl_interface_packages</member_of_group>
<export><build_type>ament_cmake</build_type></export></package>
''')
    manifest = {"ground_image": GROUND_IMAGE, "onboard_image": ONBOARD_IMAGE,
                "query_definition_sha256": hashlib.sha256(service.encode()).hexdigest(),
                "ros_domain_id": 174, "topology": "two containers on an isolated Docker bridge; native AS2 platform only; no PX4"}
    for role, image in (("ground", GROUND_IMAGE), ("onboard", ONBOARD_IMAGE)):
        manifest[role + "_image_id"] = run(["docker", "image", "inspect", image, "--format", "{{.Id}} "]).strip()
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    build = ["docker", "run", "--rm", "--network", "none", "-v", str(output) + ":/work", "-w", "/work/ws",
             "--entrypoint", "bash", GROUND_IMAGE, "-lc",
             "source /opt/ros/humble/setup.bash && colcon build --merge-install --event-handlers console_direct+"]
    (output / "build.log").write_text(run(build))
    network = "inference-baseline-" + str(os.getpid())
    onboard = network + "-onboard"
    network_created = False
    container_created = False
    try:
        run(["docker", "network", "create", network])
        network_created = True
        run(["docker", "run", "-d", "--init", "--name", onboard, "--network", network,
             "-e", "ROS_DOMAIN_ID=174", "-e", "ROS_LOCALHOST_ONLY=0", "--entrypoint", "bash", ONBOARD_IMAGE, "-lc",
             "source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && "
             "exec ros2 launch as2_platform_pixhawk pixhawk_launch.py namespace:=inference_baseline"])
        container_created = True
        result = run(["docker", "run", "--rm", "--init", "--network", network,
                      "-e", "ROS_DOMAIN_ID=174", "-e", "ROS_LOCALHOST_ONLY=0",
                      "-v", str(output) + ":/work:ro", "-v", str(ROOT) + ":/repo:ro",
                      "--entrypoint", "bash", GROUND_IMAGE, "-lc",
                      "source /opt/ros/humble/setup.bash && source /work/ws/install/setup.bash && "
                      "python3 /repo/deployment/inference/tests/as2_service_probe.py"], timeout=120)
        (output / "result.json").write_text(result)
        print(result, end="")
    except subprocess.CalledProcessError as error:
        (output / "failure.log").write_text(error.output)
        raise
    finally:
        if container_created:
            (output / "onboard.log").write_text(run(["docker", "logs", onboard]))
            run(["docker", "stop", "--time", "10", onboard], timeout=20)
            run(["docker", "rm", onboard])
        if network_created:
            run(["docker", "network", "rm", network])
        print("Evidence:", output.relative_to(ROOT))


if __name__ == "__main__":
    main()
