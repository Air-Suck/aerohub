"""Graphical editor entry: generate and validate first, then capture without Play."""
import runpy
from pathlib import Path
import unreal

scripts = Path(__file__).parent
try:
    import os
    error = Path(os.environ['COURSE_REVIEW_DIR'])/'build-error.txt'
    if error.exists():
        error.rename(error.with_name('previous-build-error.txt'))
    import json
    config=json.loads(Path(os.environ['COURSE_CONFIG_JSON']).read_text())
    if config.get('inspect_only'):
        runpy.run_path(str(scripts/'lighting_probe.py'))
    elif config.get('package_map_only'):
        runpy.run_path(str(scripts/'prepare_runtime_map.py'))
    else:
        runpy.run_path(str(scripts/('update_lighting.py' if config.get('update_mode')=='lighting_only' else 'generate_course.py')))
        runpy.run_path(str(scripts/'capture_course.py'))
except Exception:
    import traceback
    import os
    (Path(os.environ['COURSE_REVIEW_DIR'])/'build-error.txt').write_text(traceback.format_exc())
    raise
