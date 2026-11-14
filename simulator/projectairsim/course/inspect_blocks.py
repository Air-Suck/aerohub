"""Run with UE's PythonScript commandlet; exports actual placed mesh bounds only."""
import json
import os
from pathlib import Path
import unreal

output = Path(os.environ['COURSE_REVIEW_DIR'])
output.mkdir(parents=True, exist_ok=True)
unreal.EditorLoadingAndSavingUtils.load_map('/Game/BlocksMap')
actors = []
for actor in unreal.EditorLevelLibrary.get_all_level_actors():
    components = actor.get_components_by_class(unreal.StaticMeshComponent)
    if not components:
        continue
    center, extent = actor.get_actor_bounds(False)
    actors.append({
        'name': actor.get_name(), 'label': actor.get_actor_label(),
        'center_m': [center.x / 100, center.y / 100, center.z / 100],
        'extent_m': [extent.x / 100, extent.y / 100, extent.z / 100],
        'meshes': [c.static_mesh.get_path_name() if c.static_mesh else None for c in components],
        'materials': [[c.get_material(i).get_path_name() if c.get_material(i) else None
                       for i in range(c.get_num_materials())] for c in components],
    })
(output / 'blocks-inventory.json').write_text(json.dumps({
    'map': '/Game/BlocksMap', 'engine': unreal.SystemLibrary.get_engine_version(),
    'units': 'meters; UE +X,+Y,+Z; bounds are world AABBs', 'actors': actors,
}, indent=2) + '\n')
