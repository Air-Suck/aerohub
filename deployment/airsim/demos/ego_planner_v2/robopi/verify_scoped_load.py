"""Board-local load, ABI, source and interface checks; no flight commands."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import sysconfig
from ament_index_python.packages import get_package_prefix
from rclpy.serialization import serialize_message, deserialize_message
from uav_planner_interfaces.action import FollowTimedPath
from uav_ego_planner_adapter import conversion, node, scope

if len(sys.argv) != 2:
    raise SystemExit('Usage: verify_scoped_load.py <installed-candidate-root>')
root = Path(sys.argv[1]).resolve()
if root.parent != Path('/data/aerohub/validation'):
    raise SystemExit('Expected an isolated validation candidate')
manifest = json.loads((root/'deployment.json').read_text())
checks = {}
checks['all_manifest_hashes'] = all(hashlib.sha256((root/'install'/p).read_bytes()).hexdigest()==h
                                  for p,h in manifest['files'].items())
checks['target_python_abi'] = sysconfig.get_config_var('SOABI') == 'cpython-312-aarch64-linux-ohos'
checks['new_adapter_modules'] = all(Path(m.__file__).is_relative_to(root) for m in (node, conversion, scope))
checks['terminal_fix_sha'] = hashlib.sha256(Path(conversion.__file__).read_bytes()).hexdigest() == 'bbba1855ecdc68597cf49438eb3b21e1ef1f43216198b0785afef5fdbdc860e7'
native = Path(get_package_prefix('ego_planner'))/'lib/ego_planner/ego_planner_node'
data = native.read_bytes()
checks['new_native_prefix_aarch64'] = native.is_relative_to(root) and data[:4]==b'\x7fELF' and struct.unpack_from('<H',data,18)[0]==183
checks['follow_timed_path_typesupport'] = isinstance(deserialize_message(serialize_message(FollowTimedPath.Goal()),FollowTimedPath.Goal),FollowTimedPath.Goal)
result = dict(passed=all(checks.values()), checks=checks, python=sys.version,
              modules={m.__name__:m.__file__ for m in (node,conversion,scope)}, native=str(native))
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['passed'] else 1)
