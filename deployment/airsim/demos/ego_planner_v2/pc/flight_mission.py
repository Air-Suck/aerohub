"""Single-aircraft AS2 flight acceptance; simulator RPC is observation only."""
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).with_name('vendor')))
os.environ['MSGPACK_PUREPYTHON'] = '1'
from airsim_rpc import Rpc
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import Takeoff, Land
from as2_msgs.msg import PlatformInfo, PlatformStatus, ControllerInfo, TrajectorySetpoints
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleControlMode, VehicleCommandAck, VehicleCommand, TrajectorySetpoint
from std_srvs.srv import SetBool


class Mission(Node):
    def __init__(self, drone_id=0, out='/evidence', allow_peer_armed=False, require_peer=True):
        super().__init__('ego_flight_mission_' + str(drone_id), parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.drone_id = drone_id
        self.vehicle_name = 'Drone' + str(drone_id)
        self.allow_peer_armed = allow_peer_armed
        self.require_peer = require_peer
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.rpc = Rpc('host.docker.internal', 41451, .8)
        self.phase = 'preflight'
        self.latest, self.received, self.counts, self.acks = {}, {}, {}, {}
        self.keep, self.service_clients, self.handles = [], [], []
        self.error = None
        self.recovering = False
        self.origin = None
        self.truth_origin = None
        self.collision_stamp = None
        self.collision_snapshot = {'has_collided': False, 'time_stamp': 0}
        self.next_poll = 0
        self.hold_samples = []
        self.log = (self.out / 'flight.jsonl').open('w')
        self.report = {'passed': False, 'mode': 'basic', 'motion_requested': False,
                       'gates': {'takeoff_height_m': 2.0, 'takeoff_speed_m_s': .4,
                                 'hold_seconds': 5.0, 'hold_height_error_m': .3,
                                 'hold_horizontal_drift_m': .5, 'hold_speed_m_s': .3,
                                 'land_speed_m_s': .3, 'landing_height_error_m': .2,
                                 'ground_height_source': 'AirSim scene vehicle pose; PX4 estimated height drift reported separately',
                                 'data_timeout_s': 1.0, 'max_flight_speed_m_s': 2.0},
                       'actions': [], 'commands': [], 'recovery': []}
        (self.out / 'criteria.json').write_text(json.dumps(self.report['gates'], indent=2))
        topics = [
            ('platform', PlatformInfo, '/drone0/platform/info'),
            ('controller', ControllerInfo, '/drone0/controller/info'),
            ('odom', Odometry, '/drone0/navigation/odom'),
            ('cloud', PointCloud2, '/drone0/navigation/points'),
            ('mode', VehicleControlMode, '/drone0/fmu/out/vehicle_control_mode'),
            ('drone1_mode', VehicleControlMode, '/drone1/fmu/out/vehicle_control_mode'),
            ('ack', VehicleCommandAck, '/drone0/fmu/out/vehicle_command_ack_v1'),
            ('clock', Clock, '/clock'),
            ('reference_pose', PoseStamped, '/drone0/motion_reference/pose'),
            ('reference_twist', TwistStamped, '/drone0/motion_reference/twist'),
            ('reference_trajectory', TrajectorySetpoints, '/drone0/motion_reference/trajectory'),
            ('actuator', TrajectorySetpoints, '/drone0/actuator_command/trajectory'),
            ('actuator_twist', TwistStamped, '/drone0/actuator_command/twist'),
            ('px4_setpoint', TrajectorySetpoint, '/drone0/fmu/in/trajectory_setpoint')]
        for key, kind, topic in topics:
            if drone_id == 1:
                topic = topic.replace('/drone1/', '/peer/').replace('/drone0/', '/drone1/').replace('/peer/', '/drone0/')
            self.keep.append(self.create_subscription(kind, topic, lambda m, k=key: self.observe(k, m), qos_profile_sensor_data))
        self.takeoff = ActionClient(self, Takeoff, f'/drone{drone_id}/TakeoffBehavior')
        self.land = ActionClient(self, Land, f'/drone{drone_id}/LandBehavior')
        self.control_services = {name:self.create_client(SetBool, f'/drone{drone_id}/'+name)
                                 for name in ('set_arming_state','set_offboard_mode')}
        self.service_clients.extend(self.control_services.values())
        self.create_timer(.1, self.poll)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def position(self):
        p = self.latest['odom'].pose.pose.position
        return [p.x, p.y, p.z]

    def speed(self):
        v = self.latest['odom'].twist.twist.linear
        return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

    def event(self, **row):
        self.log.write(json.dumps({'wall': time.time(), 'sim': self.now_s(), 'phase': self.phase, **row}, default=float) + '\n')

    def observe(self, key, msg):
        self.latest[key] = msg
        self.received[key] = time.monotonic()
        self.counts[key] = self.counts.get(key, 0) + 1
        row = {'topic': key}
        if key == 'ack':
            self.acks[msg.command] = msg.result
            row.update(command=msg.command, result=msg.result)
        elif key in ('mode', 'drone1_mode'):
            row.update(armed=msg.flag_armed, offboard=msg.flag_control_offboard_enabled)
            if key == 'drone1_mode' and self.require_peer and msg.flag_armed and not self.allow_peer_armed:
                self.error = 'Peer vehicle unexpectedly armed'
        elif key == 'platform':
            row.update(armed=msg.armed, offboard=msg.offboard, state=msg.status.state)
        elif key == 'controller':
            row['control_mode'] = msg.input_control_mode.control_mode
        elif key == 'odom':
            row.update(position=self.position(), speed=self.speed(), frame=msg.header.frame_id,
                       stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
            if self.phase == 'holding':
                self.hold_samples.append({'position': self.position(), 'speed': self.speed()})
        elif key == 'px4_setpoint':
            row.update(position=list(msg.position), velocity=list(msg.velocity), acceleration=list(msg.acceleration), timestamp=msg.timestamp)
        elif key in ('reference_trajectory', 'actuator') and msg.setpoints:
            s = msg.setpoints[0]
            row.update(position=[s.position.x,s.position.y,s.position.z], velocity=[s.twist.x,s.twist.y,s.twist.z],
                       acceleration=[s.acceleration.x,s.acceleration.y,s.acceleration.z])
        elif key == 'reference_pose':
            p = msg.pose.position
            row.update(position=[p.x,p.y,p.z],frame=msg.header.frame_id)
        elif key in ('reference_twist', 'actuator_twist'):
            v=msg.twist.linear
            row.update(velocity=[v.x,v.y,v.z],frame=msg.header.frame_id)
        elif key in ('clock', 'cloud'):
            return
        self.event(**row)

    def poll(self):
        if time.monotonic() < self.next_poll:
            return
        self.next_poll = time.monotonic() + .25
        try:
            truth = self.rpc.call('simGetGroundTruthKinematics', self.vehicle_name)
            scene_pose = self.rpc.call('simGetVehiclePose', self.vehicle_name)
            # AirSim 1.8.1 clears the physics collision flag on this RPC.
            # Never poll it while moving: it can consume the ground contact
            # before updateRenderedState forwards it to FastPhysicsEngine.
            collision = self.collision_snapshot
            self.event(truth=truth, scene_pose=scene_pose, collision=collision, collision_fresh=False)
            if self.phase == 'landing':
                gap = abs(truth['position']['z_val']-scene_pose['position']['z_val'])
                self.report['max_landing_physics_scene_z_gap_m'] = max(gap,self.report.get('max_landing_physics_scene_z_gap_m',0.))
            self.truth = truth
            self.scene_pose = scene_pose
            p = truth['position']
            if self.truth_origin and self.phase in ('taking_off','settling','holding','ego'):
                height = self.truth_origin[2] - p['z_val']
                if height > .5 and collision['has_collided'] and collision['time_stamp'] != self.collision_stamp:
                    self.error = 'new airborne AirSim collision'
        except Exception as exc:
            self.error = 'simulator observation failed: ' + repr(exc)
        self.log.flush()

    def collision_checkpoint(self, label):
        """Only call when stationary, outside takeoff, navigation and landing."""
        if self.phase not in ('preflight','holding','ego','landed') or ('odom' in self.latest and self.speed()>.15):
            raise RuntimeError('collision checkpoint requires stationary flight boundary')
        self.collision_snapshot = self.rpc.call('simGetCollisionInfo', self.vehicle_name)
        self.event(collision_checkpoint=label, collision=self.collision_snapshot, collision_fresh=True)
        return self.collision_snapshot['time_stamp']

    def health(self):
        if self.recovering:
            return
        if self.error:
            raise RuntimeError(self.error)
        if self.origin is not None:
            # The fixed platform's ground Offboard service blocks its executor for
            # 1.2 s while prestreaming. Require live PX4/clock/simulator observations
            # during that handshake; restore all navigation gates before takeoff.
            keys = ('clock', 'mode', 'drone1_mode') if self.phase == 'arming' else (
                'odom', 'clock', 'platform', 'mode', 'controller', 'cloud', 'drone1_mode')
            for key in keys:
                if key == 'drone1_mode' and not self.require_peer:
                    continue
                if time.monotonic() - self.received.get(key, 0) > 1:
                    raise RuntimeError('stale ' + key)
            if self.phase == 'arming' and self.truth_origin:
                if abs(self.truth['position']['z_val']-self.truth_origin[2])>.15:
                    raise RuntimeError('unexpected movement during ground mode handshake')
            p = self.position()
            if not all(math.isfinite(x) for x in p) or self.speed() > 2:
                raise RuntimeError('invalid pose or flight speed envelope exceeded')
            scene = self.scene_pose['position']
            scene_height = self.truth_origin[2] - scene['z_val']
            if math.dist(p[:2], self.origin[:2]) > self.report['gates'].get('max_horizontal_excursion_m',3.) or not -.4 < scene_height < 3.2:
                raise RuntimeError('flight position envelope exceeded')
            if self.phase in ('taking_off','settling','holding','ego'):
                if not self.latest['mode'].flag_armed or not self.latest['mode'].flag_control_offboard_enabled:
                    raise RuntimeError('armed/offboard feedback lost')

    def wait(self, predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while True:
            rclpy.spin_once(self, timeout_sec=.01)
            self.health()
            if predicate():
                return
            if time.monotonic() > deadline:
                raise TimeoutError('timeout during ' + self.phase)

    def service(self, name, value, command):
        client = self.control_services[name]
        self.wait(client.service_is_ready)
        self.acks.pop(command, None)
        future = client.call_async(SetBool.Request(data=value))
        self.wait(future.done)
        if not future.result().success:
            raise RuntimeError(name + ' service rejected: ' + future.result().message)
        self.wait(lambda: command in self.acks)
        self.report['commands'].append({'service': name, 'value': value, 'ack': self.acks[command]})
        if self.acks[command] != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError(name + ' PX4 rejected: ' + str(self.acks[command]))

    def action(self, client, goal, field, timeout=60):
        self.wait(client.server_is_ready)
        future = client.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(field + ' goal rejected')
        self.handles.append(handle)
        result = handle.get_result_async()
        try:
            self.wait(result.done, timeout)
        except BaseException:
            handle.cancel_goal_async()
            raise
        record = {'result': field, 'success': bool(getattr(result.result().result, field)), 'status': result.result().status}
        self.report['actions'].append(record)
        if not record['success'] or record['status'] != 4:
            raise RuntimeError(field + ' action failed')

    def recover(self):
        self.recovering = True
        if not self.latest.get('mode') or not self.latest['mode'].flag_armed:
            return
        self.phase = 'recovery'
        if self.truth_origin and abs(self.truth['position']['z_val']-self.truth_origin[2])<.15 and self.speed()<.15:
            try:
                self.service('set_arming_state', False, 400)
                self.wait(lambda: not self.latest['mode'].flag_armed, 15)
                self.report['recovery'].append('ground-only disarm confirmed')
                return
            except Exception as exc:
                self.report['recovery'].append('ground disarm rejected; continue landing recovery: '+repr(exc))
        try:
            self.action(self.land, Land.Goal(land_speed=.3), 'land_success', 75)
            self.wait(lambda: not self.latest['mode'].flag_armed, 20)
            self.report['recovery'].append('native Land succeeded')
            return
        except Exception as exc:
            self.report['recovery'].append('native Land failed: ' + repr(exc))
        # Explicit fallback is only recovery and cannot satisfy the native-flight gate.
        pub = self.create_publisher(VehicleCommand, f'/drone{self.drone_id}/fmu/in/vehicle_command', qos_profile_sensor_data)
        self.wait(lambda: pub.get_subscription_count() > 0)
        command = VehicleCommand(timestamp=time.time_ns()//1000, command=VehicleCommand.VEHICLE_CMD_NAV_LAND,
            target_system=135+self.drone_id, target_component=1, source_system=255, source_component=1, from_external=True,
            param4=float('nan'), param5=float('nan'), param6=float('nan'), param7=float('nan'))
        self.acks.pop(command.command, None)
        pub.publish(command)
        self.wait(lambda: command.command in self.acks)
        self.report['recovery'].append({'fallback': 'PX4 NAV_LAND', 'ack': self.acks[command.command]})
        if self.acks[command.command] != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError('PX4 recovery land rejected')
        self.wait(lambda: not self.latest['mode'].flag_armed, 100)

    def run(self):
        self.wait(lambda: all(k in self.latest for k in ('platform','mode','drone1_mode','odom','cloud','controller','clock') if self.require_peer or k != 'drone1_mode')
                  and self.takeoff.server_is_ready() and self.land.server_is_ready(), 45)
        if self.latest['mode'].flag_armed or (self.require_peer and self.latest['drone1_mode'].flag_armed):
            raise RuntimeError('flight must start with both vehicles disarmed')
        self.wait(lambda: self.counts.get('odom',0)>30 and hasattr(self,'truth'))
        self.wait(lambda: all(c.service_is_ready() for c in self.control_services.values()), 10)
        # Establish response endpoints before the first side-effecting request.
        start = time.monotonic()
        self.wait(lambda: time.monotonic()-start >= 1., 5)
        self.report['control_services_preconnected'] = True
        self.origin = self.position()
        p = self.truth['position']
        self.truth_origin = [p['x_val'],p['y_val'],p['z_val']]
        self.collision_stamp = self.rpc.call('simGetCollisionInfo',self.vehicle_name)['time_stamp']
        self.report.update(origin_enu=self.origin, truth_origin_ned=self.truth_origin,
                           topology='PC simulation; separate mission client and AS2 compute containers; RoboPi not covered')
        self.phase = 'arming'
        self.report['motion_requested'] = True
        self.service('set_arming_state', True, 400)
        self.wait(lambda: self.latest['platform'].armed and self.latest['mode'].flag_armed)
        self.service('set_offboard_mode', True, 176)
        self.wait(lambda: self.latest['platform'].offboard and self.latest['mode'].flag_control_offboard_enabled)
        ready_since = [None]
        def restored():
            fresh = all(time.monotonic()-self.received.get(k,0)<.3 for k in ('odom','platform','controller','cloud','clock'))
            if not fresh:
                ready_since[0] = None
                return False
            if ready_since[0] is None:
                ready_since[0] = time.monotonic()
            return time.monotonic()-ready_since[0] >= .5
        self.wait(restored, 10)
        self.report['navigation_fresh_after_offboard'] = True
        self.phase = 'taking_off'
        self.action(self.takeoff, Takeoff.Goal(takeoff_height=2.,takeoff_speed=.4), 'takeoff_success')
        self.wait(lambda: self.latest['platform'].status.state == PlatformStatus.FLYING)
        self.phase = 'settling'
        start = self.now_s()
        self.wait(lambda: self.now_s()-start >= 3, 15)
        self.phase = 'holding'
        start = self.now_s()
        self.wait(lambda: self.now_s()-start >= 5, 20)
        if len(self.hold_samples) < 20:
            raise RuntimeError('insufficient hover samples')
        metrics = {'samples': len(self.hold_samples),
            'max_height_error_m': max(abs(r['position'][2]-self.origin[2]-2) for r in self.hold_samples),
            'max_horizontal_drift_m': max(math.dist(r['position'][:2],self.origin[:2]) for r in self.hold_samples),
            'max_speed_m_s': max(r['speed'] for r in self.hold_samples)}
        self.report['hover'] = metrics
        if metrics['max_height_error_m']>.3 or metrics['max_horizontal_drift_m']>.5 or metrics['max_speed_m_s']>.3:
            raise RuntimeError('hover acceptance failed')
        self.report['collision_observation'] = 'stationary checkpoints; no collision RPC while moving or landing'
        self.report['ego_collision_before'] = self.collision_checkpoint('before_ego')
        self.after_hover()
        self.report['ego_collision_after'] = self.collision_checkpoint('after_ego')
        if self.report['ego_collision_after'] != self.report['ego_collision_before']:
            raise RuntimeError('new collision during trajectory execution')
        self.phase = 'landing'
        self.action(self.land, Land.Goal(land_speed=.3), 'land_success', 75)
        self.wait(lambda: not self.latest['mode'].flag_armed and not self.latest['platform'].armed, 20)
        self.phase = 'landed'
        self.report['landing_estimated_height_change_m'] = self.position()[2]-self.origin[2]
        ground_pose = self.rpc.call('simGetVehiclePose', self.vehicle_name)
        self.report['landing_height_error_m'] = abs(ground_pose['position']['z_val']-self.truth_origin[2])
        if self.report['landing_height_error_m']>.2:
            raise RuntimeError('landing position not back at ground')
        if self.counts.get('px4_setpoint',0)<30 or self.counts.get('actuator',0)+self.counts.get('actuator_twist',0)<30:
            raise RuntimeError('missing actual downstream control evidence')
        self.report['passed'] = True

    def after_hover(self):
        pass


if __name__ == '__main__':
    rclpy.init()
    mission = Mission()
    try:
        mission.run()
    except BaseException as exc:
        mission.report['error'] = repr(exc)
        try:
            mission.recover()
        except BaseException as recovery:
            mission.report['recovery_error'] = repr(recovery)
    finally:
        mission.report.update(counts=mission.counts, final_phase=mission.phase,
            final_disarmed=bool(mission.latest.get('mode') and not mission.latest['mode'].flag_armed),
            drone1_disarmed=bool(mission.latest.get('drone1_mode') and not mission.latest['drone1_mode'].flag_armed))
        mission.report['passed'] = mission.report['passed'] and mission.report['final_disarmed'] and mission.report['drone1_disarmed']
        (mission.out/'result.json').write_text(json.dumps(mission.report,indent=2))
        print(json.dumps(mission.report,indent=2),flush=True)
        mission.log.close()
        mission.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if mission.report['passed'] else 1)
