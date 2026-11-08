"""Configure and execute the same C++ optimizer benchmark under each ROS version."""
from pathlib import Path
import subprocess
import sys
import time
import yaml

version, workspace, output = sys.argv[1:]
parameters = yaml.safe_load((Path(__file__).parents[1] / "humble/plan_manage/config/humble.yaml").read_text())
if version == "ros1":
    import rosgraph
    import rospy
    deadline = time.monotonic() + 20
    while True:
        try:
            rosgraph.Master("benchmark_driver").getPid()
            break
        except Exception:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.1)
    for key, value in parameters.items():
        rospy.set_param("/optimizer_benchmark/" + key.replace(".", "/"), value)
    command = [str(Path(workspace)/"devel/lib/traj_opt/optimizer_benchmark"), output]
else:
    config = Path(output).with_suffix(".yaml")
    config.write_text(yaml.safe_dump({"optimizer_benchmark": {"ros__parameters": parameters}}))
    command = ["ros2", "run", "traj_opt", "optimizer_benchmark", output,
               "--ros-args", "--params-file", str(config)]
with Path(output).with_suffix(".log").open("w") as log:
    subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=180)
print("Completed optimizer benchmark:", version, output)
