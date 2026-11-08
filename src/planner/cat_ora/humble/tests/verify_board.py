"""Run on RoboPi after loading board_env.sh, with cases and baseline alongside."""
import json, os, platform, resource, subprocess, sys, time
from pathlib import Path
root = Path(__file__).resolve().parent
prefix = Path('/data/aerohub/lib/catora/install/mrs_formation_reshaping')
processes = []
started = time.monotonic()
result = {'platform': platform.platform(), 'python': sys.version, 'domain': os.environ['ROS_DOMAIN_ID'], 'scope': 'native service and JSON proxy numerical regression; no flight commands'}
try:
    for name, command in [('node', [str(prefix/'lib/mrs_formation_reshaping/cat_ora_node')]), ('proxy', [sys.executable, str(prefix/'lib/mrs_formation_reshaping/ros2_proxy.py')])]:
        log = (root/f'{name}.log').open('w')
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        log.close()
    test = subprocess.run([sys.executable, str(root/'verify_service.py'), str(root/'baseline.json')], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
    (root/'verification.log').write_text(test.stdout)
    print(test.stdout, flush=True)
    result.update(passed=test.returncode == 0, returncode=test.returncode, processes_alive=[p.poll() is None for p in processes])
    if not all(result['processes_alive']):
        result['passed'] = False
except Exception as exc:
    result.update(passed=False, error=repr(exc))
finally:
    for process in processes:
        if process.poll() is None:
            process.terminate()
    for process in processes:
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    result['elapsed_seconds'] = time.monotonic()-started
    result['children_peak_rss_kib'] = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    (root/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)
sys.exit(0 if result['passed'] else 1)
