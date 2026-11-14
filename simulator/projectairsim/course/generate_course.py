"""UE editor Python entry point. Config is supplied as JSON through COURSE_CONFIG_JSON."""
import hashlib
import json
import math
import os
from pathlib import Path
import unreal as ue

cfg = json.loads(Path(os.environ['COURSE_CONFIG_JSON']).read_text())
out = Path(os.environ['COURSE_REVIEW_DIR'])
out.mkdir(parents=True, exist_ok=True)
TAG = 'AerohubBlocksCourseV2'
if cfg['review_status'] != 'approved' or cfg['layout'] not in ('empty','sparse_straight'):
    raise RuntimeError('Only approved supported layouts can be generated')
if cfg['target_map'] == cfg['source_map']:
    raise RuntimeError('Source map must be preserved')
project_content = Path(ue.Paths.project_content_dir())
source_file = project_content / (cfg['source_map'].removeprefix('/Game/') + '.umap')
source_hash = hashlib.sha256(source_file.read_bytes()).hexdigest()
editor = ue.get_editor_subsystem(ue.UnrealEditorSubsystem)
actors = ue.get_editor_subsystem(ue.EditorActorSubsystem)
assets = ue.EditorAssetLibrary
existing = assets.does_asset_exist(cfg['target_map'])
ue.EditorLoadingAndSavingUtils.load_map(cfg['target_map'] if existing else cfg.get('base_map',cfg['source_map']))
if existing and not any(TAG in [str(t) for t in a.tags] for a in actors.get_all_level_actors()):
    raise RuntimeError('Refusing to overwrite a target map without our ownership tag')

def managed():
    return [a for a in actors.get_all_level_actors() if TAG in [str(t) for t in a.tags]]

def signature():
    return sorted(a.get_path_name() for a in actors.get_all_level_actors() if a not in managed())

unmanaged_before = signature()
meshes = {key: ue.load_asset(cfg['assets'][key]) for key in ('cube', 'floor')}
if cfg.get('pillar_shape')=='cylinder':
    meshes['cylinder']=ue.load_asset(cfg['assets']['cylinder'])
gray = ue.load_asset(cfg['assets']['gray_material'])
green = ue.load_asset(cfg['assets']['green_material'])
if not all([*meshes.values(), gray, green]):
    raise RuntimeError('Missing required Blocks assets')
ue.load_module('StaticMeshEditor')
mesh_editor = ue.get_editor_subsystem(ue.StaticMeshEditorSubsystem)
collision_assets = {}
for key, mesh in list(meshes.items()):
    if key=='cylinder':
        # Static triangle collision follows the cylinder sides rather than a box
        # or a capsule. Only edit a course-owned copy of the engine asset.
        dest='/Game/AerohubCourse/SM_cylinder'
        copy=ue.load_asset(dest)
        if copy is None:
            copy=assets.duplicate_asset(mesh.get_path_name(),dest)
        assert copy
        body=copy.get_editor_property('body_setup')
        body.set_editor_property('collision_trace_flag',ue.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
        assets.save_loaded_asset(copy)
        meshes[key]=copy
        collision_assets[key]=copy.get_path_name()
        continue
    if mesh_editor.get_simple_collision_count(mesh) < 1:
        dest = '/Game/AerohubCourse/SM_'+key
        copy = ue.load_asset(dest)
        if copy is None:
            copy = assets.duplicate_asset(mesh.get_path_name(), dest)
        if not copy:
            raise RuntimeError('Could not duplicate collision mesh')
        if mesh_editor.get_simple_collision_count(copy) < 1:
            mesh_editor.add_simple_collisions(copy, ue.ScriptingCollisionShapeType.BOX)
        assert mesh_editor.get_simple_collision_count(copy)>0
        assets.save_loaded_asset(copy)
        meshes[key] = copy
    collision_assets[key] = meshes[key].get_path_name()
red = ue.load_asset(cfg['assets']['red_material'])
if red is None:
    red = ue.AssetToolsHelpers.get_asset_tools().create_asset('M_Red', '/Game/AerohubCourse', ue.Material, ue.MaterialFactoryNew())
    expr = ue.MaterialEditingLibrary.create_material_expression(red, ue.MaterialExpressionConstant3Vector)
    expr.set_editor_property('constant', ue.LinearColor(0.8, 0.01, 0.01, 1))
    ue.MaterialEditingLibrary.connect_material_property(expr, '', ue.MaterialProperty.MP_BASE_COLOR)
    ue.MaterialEditingLibrary.recompile_material(red)
    assets.save_loaded_asset(red)

origin = cfg['origin_m']; yaw = math.radians(cfg['yaw_deg'])
lateral_tilt=cfg.get('pillar_tilt_direction')=='lateral'
length = cfg['length_m']; width = cfg['clear_width_m']; wall = cfg['wall_thickness_m']
before = cfg['buffer_before_m']; after = cfg['buffer_after_m']
total = before+length+after; center = (length+after-before)/2

def position(s, t, z):
    return ue.Vector((origin[0]+s*math.cos(yaw)-t*math.sin(yaw))*100,
                     (origin[1]+s*math.sin(yaw)+t*math.cos(yaw))*100, (origin[2]+z)*100)

def own(actor, label):
    actor.set_actor_label('Course_'+label)
    actor.tags = [TAG]
    actor.set_folder_path('AerohubCourse/'+cfg['layout'])
    return actor

def box(label, key, s, t, z, dimensions, material, collision=True):
    mesh = meshes[key]
    bounds = mesh.get_bounding_box()
    size = bounds.max-bounds.min
    pivot = (bounds.max+bounds.min)*0.5
    scale = ue.Vector(*[dimensions[i]*100/v for i,v in enumerate((size.x,size.y,size.z))])
    offset = ue.Vector(pivot.x*scale.x, pivot.y*scale.y, pivot.z*scale.z)
    offset = ue.Vector(offset.x*math.cos(yaw)-offset.y*math.sin(yaw), offset.x*math.sin(yaw)+offset.y*math.cos(yaw), offset.z)
    a = own(actors.spawn_actor_from_class(ue.StaticMeshActor, position(s,t,z)-offset, ue.Rotator(0,cfg['yaw_deg'],0)), label)
    comp = a.static_mesh_component
    comp.set_static_mesh(mesh)
    comp.set_material(0, material)
    comp.set_collision_profile_name('BlockAll' if collision else 'NoCollision')
    a.set_actor_scale3d(scale)
    return a

def generate():
    for a in managed():
        actors.destroy_actor(a)
    box('Floor','floor',center,0,-cfg['floor_thickness_m']/2,(total,width+2*wall,cfg['floor_thickness_m']),gray)
    for side in (-1,1):
        box('Wall_'+str(side),'cube',center,side*(width+wall)/2,cfg['wall_height_m']/2,(total,wall,cfg['wall_height_m']),gray)
    for label,s,material in [('Start',0,green),('Finish',length,red)]:
        box(label,'cube',s,0,0.005,(cfg['line_width_m'],width,0.01),material,False)
    if 'split_station_m' in cfg:
        box('Join','cube',cfg['split_station_m'],0,0.005,(cfg['line_width_m'],width,0.01),red,False)
    if cfg['layout']=='sparse_straight':
        dx,dy,h=cfg['pillar_size_m']
        points=cfg['pillar_centers_st_m']
        for i,(s,t) in enumerate(points):
            assert dx/2<s<length-dx/2 and abs(t)+dy/2<width/2
            assert all(abs(s-u)>=dx or abs(t-v)>=dy for u,v in points[:i])
            a=box('Pillar_%02d'%(i+1),cfg.get('pillar_shape','cube'),s,t,h/2,(dx,dy,h),gray)
            tilt=cfg.get('pillar_tilts_deg',[0]*len(points))[i]
            if tilt:
                assert cfg['yaw_deg']==0 and cfg['pillar_shape']=='cylinder'
                bounds=meshes['cylinder'].get_bounding_box()
                pivot=(bounds.min+bounds.max)*0.5
                assert max(abs(pivot.x),abs(pivot.y),abs(pivot.z))<0.001
                theta=math.radians(tilt)
                rotation=ue.Rotator(pitch=0 if lateral_tilt else -tilt,yaw=0,roll=tilt if lateral_tilt else 0)
                a.set_actor_rotation(rotation,False)
                shift=h*math.sin(theta)/2
                a.set_actor_location(position(s if lateral_tilt else s+shift,t+shift if lateral_tilt else t,h*math.cos(theta)/2),False,False)
        ref=cfg['drone_reference']
        assert ref['scale']==1.0
        drone=[]
        for link in cfg['drone_visual_links']:
            s,t,z=link['xyz_neu_m']
            a=own(actors.spawn_actor_from_class(ue.StaticMeshActor,
                position(ref['station_m']+s,ref['lateral_m']+t,z),ue.Rotator(0,cfg['yaw_deg'],0)),
                'Drone_'+link['name'])
            mesh=ue.load_asset(link['asset'])
            assert mesh,link['asset']
            a.static_mesh_component.set_static_mesh(mesh)
            a.static_mesh_component.set_collision_profile_name('NoCollision')
            drone.append(a)
        bottom=min(a.get_actor_bounds(False)[0].z-a.get_actor_bounds(False)[1].z for a in drone)
        # Rest the complete original-scale vehicle on the green line's upper surface.
        shift=origin[2]*100+1-bottom
        for a in drone:
            a.set_actor_location(a.get_actor_location()+ue.Vector(0,0,shift),False,False)
    if cfg.get('lighting',{}).get('type')=='natural_sky':
        light=cfg['lighting']
        sun=next(a for a in actors.get_all_level_actors() if a.get_actor_label()==light['sun_actor_label'])
        sky=sun.get_component_by_class(ue.SkyLightComponent)
        directional=sun.get_component_by_class(ue.DirectionalLightComponent)
        sky.set_intensity(light['sky_intensity'])
        sky.recapture_sky()
        directional.set_intensity(light['sun_intensity'])
    if cfg.get('lighting',{}).get('type')=='point_fill':
        light=cfg['lighting']
        for i,s in enumerate(light['stations_m']):
            for j,t in enumerate(light['laterals_m']):
                a=own(actors.spawn_actor_from_class(ue.PointLight,position(s,t,light['height_m'])),
                      'Fill_%02d_%d'%(i,j))
                comp=a.get_component_by_class(ue.PointLightComponent)
                comp.set_mobility(ue.ComponentMobility.MOVABLE)
                comp.set_editor_property('use_inverse_squared_falloff',True)
                comp.set_intensity_units(ue.LightUnits.LUMENS)
                comp.set_intensity(light['intensity_lumens'])
                comp.set_attenuation_radius(light['attenuation_radius_m']*100)
                comp.set_source_radius(light['source_radius_m']*100)
                comp.set_cast_shadows(light['cast_shadows'])
                comp.set_light_color(ue.LinearColor(*[v/255 for v in light['color_rgb']],1.0))
    for label, camera in cfg['review_cameras'].items():
        loc, target = camera['location_m'], camera['target_m']
        v = ue.Vector(*[x*100 for x in loc]); aim = ue.Vector(*[x*100 for x in target])
        a = own(actors.spawn_actor_from_class(ue.CameraActor,v,ue.MathLibrary.find_look_at_rotation(v,aim)), label)
        if 'rotation_deg' in camera:
            a.set_actor_rotation(ue.Rotator(*camera['rotation_deg']),False)
        a.camera_component.set_editor_property('aspect_ratio',1280/720)
        a.camera_component.set_editor_property('field_of_view',camera.get('fov_deg',65.0))
    return sorted(a.get_actor_label() for a in managed())

first = generate()
second = generate()
expected=5+len(cfg['review_cameras'])
if 'split_station_m' in cfg:
    expected+=1
if cfg['layout']=='sparse_straight':
    expected+=len(cfg['pillar_centers_st_m'])+len(cfg['drone_visual_links'])
if cfg.get('lighting',{}).get('type')=='point_fill':
    expected+=len(cfg['lighting']['stations_m'])*len(cfg['lighting']['laterals_m'])
assert first == second and len(second)==expected
assert unmanaged_before == signature(), 'Unmanaged actor set changed'
world = editor.get_editor_world()
assert ue.EditorLoadingAndSavingUtils.save_map(world,cfg['target_map'])
ue.EditorLoadingAndSavingUtils.load_map(cfg['target_map'])
assert sorted(a.get_actor_label() for a in managed()) == second
records = []
for a in managed():
    c,e = a.get_actor_bounds(False)
    up=a.get_actor_up_vector()
    records.append({'label':a.get_actor_label(),'center_m':[c.x/100,c.y/100,c.z/100],
                    'extent_m':[e.x/100,e.y/100,e.z/100],'up_axis':[up.x,up.y,up.z]})
by_label={r['label']:r for r in records}
for label, expected in [('Course_Floor',(total,width+2*wall,cfg['floor_thickness_m'])),
                        ('Course_Wall_-1',(total,wall,cfg['wall_height_m'])),
                        ('Course_Wall_1',(total,wall,cfg['wall_height_m']))]:
    # Approved v2 is aligned with world +X; compare world AABB sizes in meters.
    assert all(abs(2*by_label[label]['extent_m'][i]-expected[i])<0.001 for i in range(3)), label
assert abs(by_label['Course_Wall_1']['center_m'][1]-wall/2-
           (by_label['Course_Wall_-1']['center_m'][1]+wall/2)-width)<0.001
if cfg['layout']=='sparse_straight':
    for i,(s,t) in enumerate(cfg['pillar_centers_st_m'],1):
        row=by_label['Course_Pillar_%02d'%i]
        dx,dy,h=cfg['pillar_size_m']
        theta=math.radians(cfg.get('pillar_tilts_deg',[0]*len(cfg['pillar_centers_st_m']))[i-1])
        expected_size=[abs(dx*math.cos(theta))+abs(h*math.sin(theta)),dy,abs(h*math.cos(theta))+abs(dx*math.sin(theta))]
        expected_center=[origin[0]+s+h*math.sin(theta)/2,origin[1]+t,origin[2]+h*math.cos(theta)/2]
        expected_up=[math.sin(theta),0,math.cos(theta)]
        if lateral_tilt:
            expected_size=[dx,abs(dy*math.cos(theta))+abs(h*math.sin(theta)),abs(h*math.cos(theta))+abs(dy*math.sin(theta))]
            expected_center=[origin[0]+s,origin[1]+t+h*math.sin(theta)/2,origin[2]+h*math.cos(theta)/2]
            expected_up=[0,math.sin(theta),math.cos(theta)]
        assert all(abs(2*row['extent_m'][j]-expected_size[j])<0.001 for j in range(3)), (i,row,expected_size)
        assert all(abs(row['center_m'][j]-expected_center[j])<0.001 for j in range(3))
        assert all(abs(row['up_axis'][j]-expected_up[j])<0.001 for j in range(3))
    pillars=[by_label['Course_Pillar_%02d'%i] for i in range(1,len(cfg['pillar_centers_st_m'])+1)]
    for i,a in enumerate(pillars):
        assert abs(a['center_m'][1]-origin[1])+a['extent_m'][1]<width/2
        for b in pillars[:i]:
            assert any(abs(a['center_m'][j]-b['center_m'][j])>a['extent_m'][j]+b['extent_m'][j] for j in (0,1)), 'Pillar bounding boxes overlap'
world = editor.get_editor_world()
hits=[]
probes=[('floor',(155,0,3),(155,0,0)),('wall_left',(155,0,4),(155,-8,4)),('wall_right',(155,0,4),(155,8,4))]
if cfg['layout']=='sparse_straight':
    for i,(s,t) in enumerate(cfg['pillar_centers_st_m'],1):
        theta=math.radians(cfg.get('pillar_tilts_deg',[0]*len(cfg['pillar_centers_st_m']))[i-1])
        y=origin[1]+t+(3*math.tan(theta) if lateral_tilt else 0)
        probes.append(('pillar_%02d'%i,(origin[0]+s-2,y,origin[2]+3),(origin[0]+s+2,y,origin[2]+3)))
for label,start,end in probes:
    hit=ue.SystemLibrary.line_trace_single(world,ue.Vector(*[v*100 for v in start]),ue.Vector(*[v*100 for v in end]),ue.TraceTypeQuery.TRACE_TYPE_QUERY1,False,[],ue.DrawDebugTrace.NONE,True)
    # UE returns a HitResult with blocking_hit exposed through break_hit_result.
    data=hit.to_tuple()
    hits.append({'probe':label,'blocking_hit':bool(data[0]),'actor':data[9].get_actor_label() if data[9] else None})
    if label.startswith('pillar_'):
        assert hits[-1]['actor']=='Course_Pillar_'+label.removeprefix('pillar_'),hits[-1]
assert all(h['blocking_hit'] and h['actor'].startswith('Course_') for h in hits), hits
radial_checks=[]
if cfg.get('pillar_shape')=='cylinder':
    for i,(s,t) in enumerate(cfg['pillar_centers_st_m'],1):
        # A vertical trace through a bounding-box corner must miss the cylinder
        # and reach the floor. This catches an accidentally retained box collision.
        theta=math.radians(cfg.get('pillar_tilts_deg',[0]*len(cfg['pillar_centers_st_m']))[i-1])
        c=by_label['Course_Pillar_%02d'%i]['center_m']
        def corner(z):
            if lateral_tilt:
                return ue.Vector((c[0]+0.45)*100,(c[1]+0.45*math.cos(theta)+z*math.sin(theta))*100,
                                 (c[2]-0.45*math.sin(theta)+z*math.cos(theta))*100)
            return ue.Vector((c[0]+0.45*math.cos(theta)+z*math.sin(theta))*100,
                             (c[1]+0.45)*100,(c[2]-0.45*math.sin(theta)+z*math.cos(theta))*100)
        hit=ue.SystemLibrary.line_trace_single(world,corner(4),corner(-4),ue.TraceTypeQuery.TRACE_TYPE_QUERY1,
            False,[],ue.DrawDebugTrace.NONE,True).to_tuple()
        assert hit[0] and hit[9].get_actor_label()=='Course_Floor'
        radial_checks.append({'pillar':i,'corner_trace_actor':hit[9].get_actor_label()})
assert hashlib.sha256(source_file.read_bytes()).hexdigest()==source_hash
drone_report=None
if cfg['layout']=='sparse_straight':
    drone=[a for a in managed() if a.get_actor_label().startswith('Course_Drone_')]
    for a in drone:
        scale=a.get_actor_scale3d()
        assert all(abs(v-1)<1e-6 for v in (scale.x,scale.y,scale.z))
    lower=[min(by_label[a.get_actor_label()]['center_m'][j]-by_label[a.get_actor_label()]['extent_m'][j] for a in drone) for j in range(3)]
    upper=[max(by_label[a.get_actor_label()]['center_m'][j]+by_label[a.get_actor_label()]['extent_m'][j] for a in drone) for j in range(3)]
    assert abs(lower[2]-origin[2]-0.01)<0.001
    drone_report={'visual_only':True,'scale':1.0,'links':cfg['drone_visual_links'],
                  'bounds_m':[lower,upper],'size_m':[upper[j]-lower[j] for j in range(3)],
                  'robot_config':cfg['drone_robot_config_name'],'robot_config_sha256':cfg['drone_robot_config_sha256']}
(out/'geometry-review.json').write_text(json.dumps({'map':cfg['target_map'],'version':cfg['version'],
    'layout':cfg['layout'],'actors':records,'collision_probes':hits,'repeat_generation_passed':True,
    'reload_passed':True,'unmanaged_actor_set_preserved':True,'collision_assets':collision_assets,
    'source_map_sha256':source_hash,'drone_reference':drone_report,
    'pillar_shape':cfg.get('pillar_shape','cube'),'radial_collision_checks':radial_checks,
    'lighting':cfg.get('lighting'),'pillar_tilts_deg':cfg.get('pillar_tilts_deg'),
    'pillar_tilt_direction':cfg.get('pillar_tilt_direction','longitudinal'),
    'section_lengths_m':cfg.get('section_lengths_m'),'split_station_m':cfg.get('split_station_m')},indent=2)+'\n')
ue.log('COURSE_GENERATION_VERIFIED')
