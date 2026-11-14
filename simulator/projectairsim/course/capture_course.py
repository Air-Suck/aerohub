"""Run via editor console `py <script>` / -ExecCmds; leave editor open after capture."""
import json
import os
import time
from pathlib import Path
import unreal as ue

out = Path(os.environ['COURSE_REVIEW_DIR'])
cfg=json.loads(Path(os.environ['COURSE_CONFIG_JSON']).read_text())
ue.EditorLoadingAndSavingUtils.load_map(cfg['target_map'])
cameras = {a.get_actor_label(): a for a in ue.get_editor_subsystem(ue.EditorActorSubsystem).get_all_level_actors()
           if isinstance(a, ue.CameraActor)}
queue = ['Course_Overview','Course_Entrance','Course_Side']
if cfg['layout']!='empty':
    queue.append('Course_DroneDetail')
if 'capture_cameras' in cfg:
    queue=['Course_'+label for label in cfg['capture_cameras']]
task = None
ready_at = time.monotonic()+30
deadline = time.monotonic()+300
completed=[]
started=time.time()

def tick(delta):
    global task, ready_at
    try:
        if time.monotonic()>deadline:
            raise RuntimeError('Screenshot deadline exceeded')
        if time.monotonic()<ready_at:
            return
        if task:
            if not task.is_task_done():
                return
            task=None
            ready_at=time.monotonic()+5
            return
        if queue:
            label=queue.pop(0)
            name=str(out/(label.lower()+'-v'+str(cfg['version'])+'.png'))
            task=ue.AutomationLibrary.take_high_res_screenshot(1280,720,name,camera=cameras[label],delay=2.0)
            if not task or not task.is_valid_task():
                raise RuntimeError('Screenshot request failed: '+label)
            completed.append(name)
        else:
            if not all(Path(p).exists() and Path(p).stat().st_mtime >= started for p in completed):
                raise RuntimeError('Missing screenshot output')
            (out/'capture-review.json').write_text(json.dumps({'map':cfg['target_map'],'version':cfg['version'],'layout':cfg['layout'],'images':completed,'mode':'editor camera screenshots; Play not started'},indent=2)+'\n')
            ue.unregister_slate_post_tick_callback(handle)
            ue.log('COURSE_CAPTURE_FINISHED: editor remains open for review')
    except Exception as exc:
        (out/'capture-error.txt').write_text(str(exc))
        ue.unregister_slate_post_tick_callback(handle)
        ue.log_error('COURSE_CAPTURE_FAILED: '+str(exc)+'; editor remains open')

handle=ue.register_slate_post_tick_callback(tick)
