"""Check actual AS2 PVA against native time, including explicitly emitted holds."""
import math
from uav_ego_planner_adapter.conversion import nanoseconds


def compare_references(sources,refs,segments,vehicle,origin_enu,goal):
    errors=[];hold_samples=0
    authorized=[]
    for source in sources:
        if not source.final(goal):continue
        p,v,a=source.sample(source.end_ns)
        for segment in segments:
            if not segment.is_final:continue
            for item in segment.vehicle_trajectories:
                if item.vehicle_id!=vehicle:continue
                start=nanoseconds(segment.valid_from)
                end=start+nanoseconds(item.points[-1].time_from_start)
                if not source.end_ns<end<=source.end_ns+100_000_000:continue
                for point in item.points:
                    if start+nanoseconds(point.time_from_start)!=source.end_ns:continue
                    vectors=[[getattr(point,k).y,getattr(point,k).x,-getattr(point,k).z]
                             for k in ('position','velocity','acceleration')]
                    if max(math.dist(x,y) for x,y in zip(vectors,(p,v,a)))<=1e-9:
                        authorized.append((source.end_ns,end,p))
    for ref in refs:
        expected=[ref['p'][i]+origin_enu[i] for i in range(3)],ref['v'],ref['a']
        candidates=[]
        for source in sources:
            if source.start_ns<=ref['stamp']<=source.end_ns:
                values=source.sample(ref['stamp'])
                candidates.append(max(math.dist(x,y) for x,y in zip(values,expected)))
        holds=[max(math.dist(p,expected[0]),math.dist([0,0,0],expected[1]),math.dist([0,0,0],expected[2]))
               for start,end,p in authorized if start<ref['stamp']<=end]
        if holds:hold_samples+=1
        errors.append(min(candidates+holds,default=math.inf))
    return max(errors,default=math.inf),hold_samples


def segment_rows(segments):
    return [dict(start_ns=nanoseconds(s.valid_from),is_final=s.is_final,
        vehicles=[dict(vehicle_id=item.vehicle_id,points=[dict(
            t_ns=nanoseconds(p.time_from_start),p=[p.position.y,p.position.x,-p.position.z],
            v=[p.velocity.y,p.velocity.x,-p.velocity.z],a=[p.acceleration.y,p.acceleration.x,-p.acceleration.z])
            for p in item.points]) for item in s.vehicle_trajectories]) for s in segments]
