"""Keep one editor open; consume local review requests on its main thread."""
import json
import os
import runpy
import time
from pathlib import Path
import unreal as ue

root=Path(os.environ['COURSE_SESSION_DIR'])
root.mkdir(parents=True,exist_ok=True)
scripts=Path(__file__).parent
ue.EditorLoadingAndSavingUtils.load_map('/Game/BlocksCourseSparse')
last_id=None
next_poll=0

def inspect():
    runpy.run_path(str(scripts/'lighting_probe.py'))

def tick(delta):
    global last_id,next_poll
    if time.monotonic()<next_poll:return
    next_poll=time.monotonic()+1
    request=root/'request.json'
    if not request.exists():return
    try:
        job=json.loads(request.read_text())
        if job['id']==last_id:return
        last_id=job['id']
        if job['action']=='inspect':inspect()
        elif job['action'] in ('build','capture'):
            os.environ['COURSE_REVIEW_DIR']=job['output']
            os.environ['COURSE_CONFIG_JSON']=job['config']
            Path(job['output']).mkdir(parents=True,exist_ok=True)
            runpy.run_path(str(scripts/('build_and_capture.py' if job['action']=='build' else 'capture_course.py')))
        else:raise ValueError('Unsupported review action')
        (root/'status.json').write_text(json.dumps({'id':last_id,'status':'submitted' if job['action']!='inspect' else 'complete'}))
    except Exception:
        import traceback
        (root/'status.json').write_text(json.dumps({'id':last_id,'status':'error','detail':traceback.format_exc()}))

handle=ue.register_slate_post_tick_callback(tick)
inspect()
ue.log('COURSE_LIVE_SESSION_READY: editor remains open')
