"""Adjust environment lighting in the current editor world; preserve all geometry."""
import hashlib
import json
import os
from pathlib import Path
import unreal as ue

cfg=json.loads(Path(os.environ['COURSE_CONFIG_JSON']).read_text())
out=Path(os.environ['COURSE_REVIEW_DIR'])
actors=ue.get_editor_subsystem(ue.EditorActorSubsystem)
world=ue.get_editor_subsystem(ue.UnrealEditorSubsystem).get_editor_world()
assert world.get_path_name().split('.')[0]==cfg['target_map']

def geometry():
    result=[]
    for a in actors.get_all_level_actors():
        for c in a.get_components_by_class(ue.StaticMeshComponent) or []:
            if c.static_mesh:
                tf=c.get_world_transform()
                p,q,s=tf.translation,tf.rotation,tf.scale3d
                result.append((a.get_path_name(),c.static_mesh.get_path_name(),
                               (p.x,p.y,p.z,q.x,q.y,q.z,q.w,s.x,s.y,s.z)))
    return sorted(result)

before=geometry()
light=cfg['lighting']
assert light['type']=='natural_sky'
sun=next(a for a in actors.get_all_level_actors() if a.get_actor_label()==light['sun_actor_label'])
sky=sun.get_component_by_class(ue.SkyLightComponent)
directional=sun.get_component_by_class(ue.DirectionalLightComponent)
sky.set_intensity(light['sky_intensity'])
sky.recapture_sky()
directional.set_intensity(light['sun_intensity'])
assert before==geometry()
assert not any(a.get_actor_label().startswith('Course_Fill_') for a in actors.get_all_level_actors())
assert ue.EditorLoadingAndSavingUtils.save_map(world,cfg['target_map'])
(out/'lighting-review.json').write_text(json.dumps({'map':cfg['target_map'],'lighting':light,
    'geometry_unchanged':True,'point_fill_count':0,'editor_restarted':False},indent=2)+'\n')
