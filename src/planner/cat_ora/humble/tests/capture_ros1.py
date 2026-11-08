"""Capture real ROS 1 responses; requires the upstream service running."""
import importlib.util
import json
import sys
from pathlib import Path

import rospy

root = Path(__file__).resolve().parents[5]
spec = importlib.util.spec_from_file_location("proxy", root / "thirdparty/cat_ora_ros1/aerohub/ros1_proxy.py")
proxy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(proxy)
rospy.init_node("capture_catora_baseline")
rospy.wait_for_service(proxy.SERVICE, timeout=30)
responses = {}
for case in json.loads(Path(__file__).with_name("cases.json").read_text()):
    responses[case["name"]] = proxy.generate(case)["trajectories_enu"]
    if case["name"] == "unequal_constraints":
        # Reference for the intentional argument-order correction, separately from legacy behavior.
        corrected = dict(case, max_velocity=case["max_acceleration"], max_acceleration=case["max_velocity"])
        responses["unequal_constraints_corrected"] = proxy.generate(corrected)["trajectories_enu"]
Path(sys.argv[1]).write_text(json.dumps(responses))
print("Captured five ROS 1 cases and the corrected-constraint reference")
