"""Inspect lighting and leave the live viewport at the course entrance."""
import json
import os
from pathlib import Path
import unreal as ue
root=Path(os.environ['COURSE_SESSION_DIR'])

def inspect():
    rows=[]
    for actor in ue.get_editor_subsystem(ue.EditorActorSubsystem).get_all_level_actors():
        comps=list(actor.get_components_by_class(ue.LightComponentBase) or [])
        comps+=list(actor.get_components_by_class(ue.PostProcessComponent) or [])
        comps+=list(actor.get_components_by_class(ue.CameraComponent) or [])
        if not comps: continue
        entries=[]
        for c in comps:
            props={}
            for key in ['intensity','light_color','mobility','cast_shadows','source_type','real_time_capture',
                        'lower_hemisphere_is_black','indirect_lighting_intensity','auto_exposure_bias',
                        'use_inverse_squared_falloff','intensity_units']:
                try: props[key]=str(c.get_editor_property(key))
                except Exception: pass
            try:
                settings=c.get_editor_property('post_process_settings')
                for k in ['auto_exposure_method','auto_exposure_min_brightness','auto_exposure_max_brightness',
                          'auto_exposure_bias','override_auto_exposure_bias']:
                    props[k]=str(settings.get_editor_property(k))
            except Exception: pass
            entries.append({'class':c.get_class().get_name(),'properties':props})
        rows.append({'label':actor.get_actor_label(),'class':actor.get_class().get_name(),'components':entries})
    (root/'lighting-inventory.json').write_text(json.dumps(rows,indent=2)+'\n')

inspect()
all_actors=ue.get_editor_subsystem(ue.EditorActorSubsystem).get_all_level_actors()
view=next((a for a in all_actors if a.get_actor_label()=='Course_Stage2Entrance'),None)
if view is None:
    view=next((a for a in all_actors if a.get_actor_label()=='Course_Entrance'),None)
if view:
    ue.EditorLevelLibrary.set_level_viewport_camera_info(view.get_actor_location(),view.get_actor_rotation())
