"""Save a runtime-only copy; keep visual scale references out of cooked gameplay."""
import hashlib
import json
import os
from pathlib import Path
import unreal as ue

out=Path(os.environ['COURSE_REVIEW_DIR'])
cfg=json.loads(Path(os.environ['COURSE_CONFIG_JSON']).read_text())
source=cfg['review_map']
target=cfg['runtime_map']
assert source!=target
content=Path(ue.Paths.project_content_dir())
source_file=content/(source.removeprefix('/Game/')+'.umap')
before=hashlib.sha256(source_file.read_bytes()).hexdigest()
ue.EditorLoadingAndSavingUtils.load_map(source)
actors=ue.get_editor_subsystem(ue.EditorActorSubsystem)
reference=[]
obsolete_captures=[]
frame=None
for a in actors.get_all_level_actors():
    label=a.get_actor_label()
    if a.get_components_by_class(ue.ReflectionCaptureComponent):
        # These inherited captures describe the original building, not the new
        # corridor. Runtime reflections use the accepted live SkyLight instead.
        a.set_editor_property('is_editor_only_actor',True)
        obsolete_captures.append(label)
    if label.startswith('Course_Drone_'):
        a.set_editor_property('is_editor_only_actor',True)
        reference.append(label)
        if label=='Course_Drone_Frame':
            p=a.get_actor_location()
            frame=[p.x/100,p.y/100,p.z/100]
    if isinstance(a,ue.CameraActor):
        a.set_editor_property('is_editor_only_actor',label!='Course_Entrance')
        if label=='Course_Entrance':
            a.set_editor_property('auto_activate_for_player',ue.AutoReceiveInput.PLAYER0)
assert len(reference)==5 and frame
world=ue.get_editor_subsystem(ue.UnrealEditorSubsystem).get_editor_world()
assert ue.EditorLoadingAndSavingUtils.save_map(world,target)
assert hashlib.sha256(source_file.read_bytes()).hexdigest()==before
(out/'runtime-map.json').write_text(json.dumps({'review_map':source,'runtime_map':target,
    'review_map_sha256':before,'editor_only_reference_actors':reference,
    'editor_only_obsolete_reflection_captures':obsolete_captures,
    'robot_spawn_ue_m':frame,'robot_spawn_native_ned_m':[frame[0],frame[1],-frame[2]],
    'startup_camera':'Course_Entrance'},indent=2)+'\n')
# Return to the user's accepted review map; leave the editor running.
ue.EditorLoadingAndSavingUtils.load_map(source)
ue.log('COURSE_RUNTIME_MAP_SAVED: editor remains open')
