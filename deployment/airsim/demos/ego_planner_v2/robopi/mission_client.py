"""Reuse flight criteria with explicit one-vehicle scope; no algorithm subprocesses."""
import json
import math
from pathlib import Path
import sys
sys.path.insert(0, '/input')
import rclpy
from flight_mission import Mission
from ego_flight_mission import EgoMission
from ego_reference_check import segment_rows
from interfaces import topic_map


def main():
    config = json.loads(Path('/session/config.json').read_text())
    mapping = topic_map(config)
    remaps = [arg for old, new in mapping.items() for arg in ('-r', old+':='+new)]
    rclpy.init(args=['--ros-args', *remaps] if remaps else [])
    if sys.argv[1] == 'fault':
        from fault_mission import FaultMission
        config = json.loads(Path('/session/config.json').read_text())
        task = FaultMission(config['fault_case'])
    else:
        task = EgoMission(require_peer=False) if sys.argv[1] == 'ego' else Mission(require_peer=False)
    # The active UE scene can retain Drone1 without a second PX4 instance.
    # Observe that vehicle via simulator RPC; do not invent ROS disarm feedback.
    peers = [name for name in task.rpc.call('listVehicles') if name != task.vehicle_name]
    peer_origins = {}
    for name in peers:
        p = task.rpc.call('simGetVehiclePose', name)['position']
        peer_origins[name] = [p[k] for k in ('x_val','y_val','z_val')]
    task.report['inactive_vehicles'] = dict(origins=peer_origins, max_displacement_m=0., limit_m=.15)
    task.report['interface_mapping'] = mapping
    # The timer already holds the original bound method. Observe peers from health,
    # throttled by monotonic time, before issuing commands and throughout the flight.
    import time
    next_peer_poll = [0.]
    original_health = task.health
    def health():
        if not task.recovering and time.monotonic() >= next_peer_poll[0]:
            next_peer_poll[0] = time.monotonic()+.5
            for name, origin in peer_origins.items():
                pose = task.rpc.call('simGetVehiclePose', name)
                p = pose['position']
                displacement = math.dist(origin, [p[k] for k in ('x_val','y_val','z_val')])
                task.event(inactive_vehicle=name, scene_pose=pose, displacement_m=displacement)
                task.report['inactive_vehicles']['max_displacement_m'] = max(displacement, task.report['inactive_vehicles']['max_displacement_m'])
                if displacement > .15:
                    raise RuntimeError('Inactive vehicle moved: '+name)
            if task.latest.get('drone1_mode') and task.latest['drone1_mode'].flag_armed:
                raise RuntimeError('Inactive peer unexpectedly armed')
        if not task.recovering and Path('/evidence/recovery.request').exists():
            raise RuntimeError('Deployment monitor requested recovery: '+Path('/evidence/recovery.request').read_text())
        return original_health()
    task.health = health
    try:
        task.run()
    except BaseException as exc:
        task.report['error'] = repr(exc)
        try:
            task.recover()
        except BaseException as recovery:
            task.report['recovery_error'] = repr(recovery)
    finally:
        task.report.update(counts=task.counts, final_phase=task.phase,
            final_disarmed=bool(task.latest.get('mode') and not task.latest['mode'].flag_armed),
            peer_required=False, topology='RoboPi single compute; PC simulator, PX4 and ROS mission client')
        task.report['passed'] = task.report['passed'] and task.report['final_disarmed'] and 'error' not in task.report
        if isinstance(task, EgoMission):
            task.report['ego_state'] = task.ego_state
            for name, data in [('polynomials', task.poly_records), ('references', task.ego_refs),
                               ('segments', segment_rows(task.segments))]:
                (task.out/(name+'.json')).write_text(json.dumps(data, default=float))
        if hasattr(task, 'timeline'):
            task.report['final_injector'] = task.injector
            (task.out/'fault-timeline.json').write_text(json.dumps(task.timeline, default=float))
        (task.out/'result.json').write_text(json.dumps(task.report, indent=2, default=float))
        task.log.close()
        task.destroy_node()
        rclpy.try_shutdown()
    return 0 if task.report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
