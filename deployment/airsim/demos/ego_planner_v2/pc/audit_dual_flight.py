"""Independent raw-log audit and plots; run offline after disarm."""
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path('/evidence')
report=json.loads((root/'result.json').read_text())
audit={'passed':False,'vehicles':{},'scope':'PC simulation; sampled separation and scene observations; no crossing avoidance or RoboPi claim'}
logs={};tracks={};scene_tracks={};checks=[report['passed'],report['both_disarmed'],not report['recovery']]
execution_checks=[]

def evaluate(source,stamp):
    ends=np.cumsum(source['duration']);elapsed=(stamp-source['start_ns'])/1e9
    knots=[source['start_ns']+round(float(t)*1e9) for t in ends]
    if stamp in knots:
        index=knots.index(stamp);t=source['duration'][index]
    else:
        index=min(int(np.searchsorted(ends,elapsed,side='right')),len(ends)-1)
        t=elapsed-(ends[index-1] if index else 0.)
    coefficients=np.array([source[k][6*index:6*(index+1)] for k in ('coef_x','coef_y','coef_z')])
    return [np.array([np.polyval(np.polyder(c,d),t) for c in coefficients]) for d in (0,1,2)]

def reference_errors(polys,refs,goal,common_end):
    errors=[]
    for ref in refs:
        options=[];stamp=ref['stamp_ns']
        for p in polys:
            end=p['start_ns']+round(sum(p['duration'])*1e9)
            if p['start_ns']<=stamp<=end:
                expected=evaluate(p,stamp)
            elif end<stamp<=common_end:
                expected=evaluate(p,end)
                if np.linalg.norm(expected[0]-goal)>.03 or max(np.linalg.norm(x) for x in expected[1:])>1e-4:continue
                expected=[expected[0],np.zeros(3),np.zeros(3)]
            else:continue
            options.append(max(math.dist(value,ref[key]) for value,key in zip(expected,('position','velocity','acceleration'))))
        errors.append(min(options,default=math.inf))
    return max(errors,default=math.inf)

all_polys=[p for i in (0,1) for p in json.loads((root/f'drone{i}'/'polynomials.json').read_text())]
common_end=max((p['start_ns']+round(sum(p['duration'])*1e9) for p in all_polys),default=0)
for i in (0,1):
    folder=root/f'drone{i}'
    summary=json.loads((folder/'result.json').read_text())
    rows=[json.loads(line) for line in (folder/'flight.jsonl').read_text().splitlines()]
    logs[i]=rows
    # ROS use_sim_time starts at zero until the first /clock callback. These
    # preflight samples cannot define an epoch-spanning interpolation grid.
    odom=[r for r in rows if r.get('topic')=='odom' and r['sim']>0 and abs(r['sim']-r['stamp'])<1.]
    tracks[i]=np.array([[r['sim'],r['position'][0]-4*i,*r['position'][1:],r['speed']] for r in odom])
    scenes=[r for r in rows if 'scene_pose' in r and r['sim']>0]
    scene_tracks[i]=np.array([[r['sim'],r['scene_pose']['position']['y_val']-4*i,
        r['scene_pose']['position']['x_val'],-r['scene_pose']['position']['z_val']] for r in scenes])
    if i not in report['active']:
        checks.append(not any(r.get('topic')=='mode' and r.get('armed') for r in rows))
        audit['vehicles'][f'drone{i}']={'inactive_never_armed':checks[-1]}
        continue
    checks.extend([summary['passed'],summary['final_disarmed'],not summary['recovery'],
        len(summary['actions'])==2,all(a['status']==4 and a['success'] for a in summary['actions'])])
    phase='ego' if report['stage']=='dual' else 'holding'
    begin=next(r['wall'] for r in rows if r['phase']==phase)
    before=[r for r in scenes if r['wall']<begin]
    observed=[r for r in scenes if r['phase']==phase]
    baseline=before[-1]['collision']['time_stamp']
    new=sorted({r['collision']['time_stamp'] for r in observed if r['collision']['time_stamp']!=baseline})
    if report.get('collision_observation') and report['stage']=='dual':
        checkpoints={r['collision_checkpoint']:r for r in rows if 'collision_checkpoint' in r}
        baseline=checkpoints['before_ego']['collision']['time_stamp']
        after=checkpoints['after_ego']['collision']['time_stamp']
        new=[] if after==baseline else [after]
        checks.extend([checkpoints['before_ego']['wall']<=begin,
            checkpoints['after_ego']['wall']>=max(r['wall'] for r in rows if r['phase']=='ego' and r.get('topic')=='reference_trajectory'),
            summary.get('max_landing_physics_scene_z_gap_m',math.inf)<=.3])
    checks.append(len(observed)>=5 and not new)
    values=dict(collision_observations=2 if report.get('collision_observation') else len(observed),
        scene_observations=len(observed),collision_stamp_before=baseline,
        new_collision_timestamps=new,landing_height_error_m=summary.get('landing_height_error_m'),
        collision_observation=report.get('collision_observation','legacy polling'),
        max_landing_physics_scene_z_gap_m=max((abs(r['truth']['position']['z_val']-r['scene_pose']['position']['z_val'])
            for r in scenes if r['phase']=='landing'),default=0.),
        max_recovery_physics_scene_z_gap_m=max((abs(r['truth']['position']['z_val']-r['scene_pose']['position']['z_val'])
            for r in scenes if r['phase']=='recovery'),default=0.))
    if report['stage']=='dual':
        flight=[r for r in odom if r['phase']=='ego']
        actual=np.array([r['position'] for r in flight])
        start=np.array(summary['ego_start_position']);goal=np.array(summary['goal_enu'])
        direction=goal-start
        along=np.clip((actual-start)@direction/np.dot(direction,direction),0,1)
        tube=np.linalg.norm(actual-(start+along[:,None]*direction),axis=1)
        refs=json.loads((folder/'references.json').read_text())
        polys=json.loads((folder/'polynomials.json').read_text())
        max_v=max(math.dist(r['velocity'],[0,0,0]) for r in refs)
        max_a=max(math.dist(r['acceleration'],[0,0,0]) for r in refs)
        speed=max(r['speed'] for r in flight)
        end_error=float(np.linalg.norm(actual[-1]-goal))
        pva_error=reference_errors(polys,refs,np.array(summary['goal_world_enu']),common_end)
        values.update(goal_error_m=end_error,actual_speed_max_m_s=speed,
            reference_speed_max_m_s=max_v,reference_acceleration_max_m_s2=max_a,
            max_route_tube_error_m=float(tube.max()),
            actual_displacement_m=float(np.linalg.norm(actual[-1]-actual[0])),
            actual_path_length_m=float(np.linalg.norm(np.diff(actual,axis=0),axis=1).sum()),
            polynomial_count=len(polys),reference_count=len(refs),
            max_pva_error=pva_error)
        vehicle_execution_checks=[end_error<=.25,speed<=.5,max_v<=.2415+1e-8,max_a<=.3675+1e-8,
            tube.max()<=.5,values['actual_displacement_m']>=.75,
            pva_error<=1e-4,
            all(p['drone_id']==i for p in polys),len(polys)>0,len(refs)>=20,
            all(r['frame']==f'drone{i}/odom' for r in odom),len(observed)>=5,not new,
            summary['ego_downstream_counts']['px4_setpoint']>=20,summary['ego_downstream_counts']['actuator']>=20]
        checks.extend(vehicle_execution_checks);execution_checks.extend(vehicle_execution_checks)
    audit['vehicles'][f'drone{i}']=values

def aligned(data,step,max_gap):
    start=max(t[0,0] for t in data.values());end=min(t[-1,0] for t in data.values())
    if not 0<end-start<600:raise ValueError('invalid or unexpectedly long audit time span')
    grid=np.arange(start,end,step)
    outputs=[];mask=np.ones(len(grid),dtype=bool)
    for i in (0,1):
        t=data[i]
        idx=np.clip(np.searchsorted(t[:,0],grid),1,len(t)-1)
        mask&=(t[idx,0]-t[idx-1,0])<=max_gap
        outputs.append(np.column_stack([np.interp(grid,t[:,0],t[:,axis]) for axis in range(1,t.shape[1])]))
    return grid[mask],[v[mask] for v in outputs],int((~mask).sum())

grid,positions,missing=aligned(tracks,.02,.1)
separation=np.linalg.norm(positions[0][:,:3]-positions[1][:,:3],axis=1)
sg,scene,scene_missing=aligned(scene_tracks,.1,.6)
scene_separation=np.linalg.norm(scene[0]-scene[1],axis=1)
audit.update(minimum_odom_separation_m=float(separation.min()),minimum_scene_separation_m=float(scene_separation.min()),
    odom_gap_exclusions=missing,scene_gap_exclusions=scene_missing)
checks.extend([len(grid)>100,separation.min()>=2,scene_separation.min()>=2])
if report['stage']=='dual':
    begin=max(min(r['sim'] for r in logs[i] if r['phase']=='ego') for i in (0,1))
    end=min(max(r['sim'] for r in logs[i] if r['phase']=='ego') for i in (0,1))
    moving=(grid>=begin)&(grid<=end)&(positions[0][:,3]>.05)&(positions[1][:,3]>.05)
    audit['simultaneous_motion_sim_s']=float(moving.sum()*.02)
    checks.append(audit['simultaneous_motion_sim_s']>=.5)
    checks.extend([report['ego_state']['state']=='SUCCEEDED',report['ego_state']['stop_confirmed'],
        sorted(report['ego_state']['completed'])==['drone0','drone1'],report['ego_state']['counts']['goals']==2])
    execution_checks.extend([audit['simultaneous_motion_sim_s']>=.5,separation.min()>=2,scene_separation.min()>=2,
        report['ego_state']['state']=='SUCCEEDED',report['ego_state']['stop_confirmed'],
        sorted(report['ego_state']['completed'])==['drone0','drone1'],report['ego_state']['counts']['goals']==2])
    audit['ego_execution_passed']=all(bool(v) for v in execution_checks)
audit['checks_passed']=sum(bool(v) for v in checks);audit['checks_total']=len(checks)
audit['passed']=all(bool(v) for v in checks)
(root/'independent-audit.json').write_text(json.dumps(audit,indent=2))

fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
origin=min(t[0,0] for t in tracks.values())
colors=['#1874bd','#d86f17']
for i in (0,1):
    t=tracks[i];label=f'Drone{i}';color=colors[i]
    axes[0,0].plot(t[:,1],t[:,2],color=color,label=label)
    axes[0,0].scatter(t[0,1],t[0,2],color=color,marker='o')
    axes[0,0].scatter(t[-1,1],t[-1,2],color=color,marker='x')
    axes[0,1].plot(t[:,1]-t[0,1],t[:,2]-t[0,2],color=color,label=label)
    axes[1,0].plot(t[:,0]-origin,t[:,3]-t[0,3],color=color,label=label)
    scene=scene_tracks[i]
    axes[1,0].plot(scene[:,0]-origin,scene[:,3]-scene[0,3],color=color,ls='--',label=label+' scene height')
    axes[1,1].plot(t[:,0]-origin,t[:,4],color=color,label=label)
axes[0,0].set(title='Actual tracks in fleet ENU',xlabel='East (m)',ylabel='North (m)',aspect='equal')
axes[0,1].set(title='Displacement from each takeoff position',xlabel='East displacement (m)',ylabel='North displacement (m)',ylim=(-.5,.5),aspect='equal')
axes[1,0].set(title='Estimated vs scene height (landing divergence)',xlabel='Simulation time (s)',ylabel='Height relative to takeoff (m)')
axes[1,1].set(title='Localization speed',xlabel='Simulation time (s)',ylabel='Speed (m/s)')
if report['stage']=='dual':
    for ax in axes[1,:]:ax.axvspan(begin-origin,end-origin,color='#34a853',alpha=.10,label='EGO phase')
    axes[1,1].plot([begin-origin,end-origin],[.5,.5],color='gray',ls='--',label='EGO phase speed envelope')
for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.suptitle(('Dual independent EGO/AS2 flight' if report['stage']=='dual' else 'Drone1 basic AS2 flight')+
    (' — overall PASS' if audit['passed'] else ' — overall FAIL (landing/recovery)'))
fig.savefig(root/'trajectory.png',dpi=170);fig.savefig(root/'trajectory.pdf')
print(json.dumps(audit,indent=2))
raise SystemExit(0 if audit['passed'] else 1)
