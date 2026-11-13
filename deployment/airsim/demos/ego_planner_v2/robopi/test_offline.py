"""Offline lifecycle/packaging contracts: no ROS, HDC, Docker or subprocess startup."""
import ast
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch
import run
from policy import safe_to_stop, capability_ready, run_id


class Contracts(unittest.TestCase):
    def test_hold_audit_rejects_unrecorded_nonfinal_or_unbounded_extension(self):
        tree=ast.parse((run.HERE/'audit_ego.py').read_text())
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='authorized_holds')
        ns={'math':math,'sample':lambda p,t,d: [1.,2.,3.] if d==0 else [0.,0.,0.]}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<independent hold authorization>','exec'),ns)
        poly={'start_ns':1_000_000_000,'duration':[1.]}
        points=[dict(t_ns=100_000_000,p=[1.,2.,3.],v=[0.,0.,0.],a=[0.,0.,0.]),
                dict(t_ns=200_000_000,p=[1.,2.,3.],v=[0.,0.,0.],a=[0.,0.,0.])]
        segment=dict(start_ns=1_900_000_000,is_final=True,vehicles=[dict(vehicle_id='drone0',points=points)])
        check=lambda rows,goal=[1.,2.,3.]:ns['authorized_holds']([poly],rows,goal)
        self.assertEqual(len(check([segment])),1)
        self.assertFalse(check([]))
        self.assertFalse(check([dict(segment,is_final=False)]))
        self.assertFalse(check([segment],[8.,2.,3.]))
        bad=copy.deepcopy(segment);bad['vehicles'][0]['points'][-1]['t_ns']+=1
        self.assertFalse(check([bad]))
        bad=copy.deepcopy(segment);bad['vehicles'][0]['points'][0]['t_ns']+=1
        self.assertFalse(check([bad]))
        bad=copy.deepcopy(segment);bad['vehicles'][0]['points'][-1]['v'][0]=.01
        self.assertFalse(check([bad]))

    def test_candidate_scope_mismatch_is_rejected(self):
        config=json.loads((run.HERE/'config.example.json').read_text())
        config.update(candidate='/data/aerohub/validation/ego-v2-20261112',vehicle='')
        scoped=dict(config,candidate='/data/aerohub/validation/ego-v2-scoped-20261113',vehicle='drone0')
        run.validate(scoped)
        for invalid in (dict(scoped,vehicle=''),dict(config,vehicle='drone0'),dict(scoped,vehicle='drone1')):
            with self.assertRaises(ValueError): run.validate(invalid)
        self.assertEqual(run.topic_map(scoped)['/ego/cancel'],'/drone0/ego/cancel')
        self.assertEqual(run.topic_map(config),{})

    def test_scoped_prepare_carries_same_mapping_on_board(self):
        config=json.loads((run.HERE/'config.example.json').read_text())
        config.update(candidate='/data/aerohub/validation/ego-v2-scoped-20261113',vehicle='drone0')
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'scoped-01';run.prepare(out,config)
            self.assertEqual((out/'bundle/interfaces.py').read_bytes(),(run.HERE/'interfaces.py').read_bytes())
            self.assertEqual(json.loads((out/'bundle/config.json').read_text())['vehicle'],'drone0')

    def test_planner_kill_requires_exact_session_identity(self):
        tree=ast.parse((run.HERE/'board_fault_injector.py').read_text())
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='owned')
        namespace={}
        exec(compile(ast.Module(body=[function],type_ignores=[]),'<kill ownership>','exec'),namespace)
        row=dict(argv=['/candidate/planner','__ns:=/drone0'],parent=80,domain=['ROS_DOMAIN_ID=96'])
        self.assertTrue(namespace['owned'](row,'/candidate/planner',80,96))
        for changed in (dict(parent=81),dict(domain=['ROS_DOMAIN_ID=97']),dict(argv=['/other/planner','__ns:=/drone0']),dict(argv=['/candidate/planner','__ns:=/drone1'])):
            self.assertFalse(namespace['owned'](dict(row,**changed),'/candidate/planner',80,96))

    def test_fault_prepare_only_adds_explicit_test_bundle(self):
        config=json.loads((run.HERE/'config.example.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'fault-01'
            run.prepare(out,dict(config,fault_case='cloud'))
            self.assertTrue((out/'bundle/fault.launch.py').exists())
            self.assertTrue((out/'bundle/fault_injector_base.py').exists())
            self.assertEqual(json.loads((out/'config.json').read_text())['fault_case'],'cloud')
            with self.assertRaisesRegex(ValueError,'Unknown fault'):
                run.validate(dict(config,fault_case='arbitrary'))

    def test_audit_distinguishes_sensor_rejections_and_navigation_counts(self):
        tree = ast.parse((run.HERE/'audit_ego.py').read_text())
        functions = [n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('planning_rejections','execution_counts_valid')]
        namespace = {}
        exec(compile(ast.Module(body=functions,type_ignores=[]), '<audit counter semantics>', 'exec'),namespace)
        rejected = namespace['planning_rejections']
        self.assertEqual(rejected({'odom_stamp':1,'cloud_stamp':2}), {})
        self.assertEqual(rejected({'odom_stamp':1,'duplicate_trajectory':1}), {'duplicate_trajectory':1})
        self.assertEqual(rejected({'unknown_rejection':1}), {'unknown_rejection':1})
        valid = namespace['execution_counts_valid']
        state = dict(counts=dict(trajectories=8,segments=7),revision=7)
        self.assertTrue(valid(state,8))
        self.assertFalse(valid(state,7))
        self.assertFalse(valid(dict(counts=dict(trajectories=8,segments=7),revision=6),8))

    def test_hold_reset_refuses_an_armed_vehicle(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(run, 'execute', return_value='Armed\nnavigation mode: Offboard') as call:
            with self.assertRaisesRegex(RuntimeError, 'requires confirmed PX4 Disarmed'):
                run.restore_ground_hold(Path(tmp), Path(tmp))
            self.assertEqual(call.call_count, 1)

    def test_hold_reset_requires_health_and_actual_mode(self):
        replies = ['Disarmed\nnavigation mode: Offboard', '', 'Disarmed\nnavigation mode: Hold', 'pre_flight_checks_pass: True']
        with tempfile.TemporaryDirectory() as tmp, patch.object(run, 'execute', side_effect=replies) as call:
            run.restore_ground_hold(Path(tmp), Path(tmp))
            self.assertEqual(call.call_args_list[1].args[0][-2:], ['mode','auto:loiter'])
            self.assertEqual(len(json.loads((Path(tmp)/'ground-hold.json').read_text())), 4)

    def test_cancel_failure_still_attempts_native_landing(self):
        tree = ast.parse((run.PROJECT/'deployment/airsim/demos/ego_planner_v2/pc/ego_flight_mission.py').read_text())
        definition = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'EgoMission')
        definition.body = [n for n in definition.body if isinstance(n, ast.FunctionDef) and n.name == 'recover']
        class Base:
            def recover(self):
                self.landing_attempted = True
        namespace = {'Mission':Base}
        exec(compile(ast.Module(body=[definition], type_ignores=[]), '<actual recovery method>', 'exec'), namespace)
        task = namespace['EgoMission']()
        task.phase = 'ego'
        task.report = {'recovery':[]}
        task.cancel_client = type('Client', (), {'service_is_ready':lambda self:False})()
        task.wait = lambda *_: (_ for _ in ()).throw(TimeoutError('cancel unavailable'))
        task.recover()
        self.assertTrue(task.landing_attempted)
        self.assertIn('cancel unavailable', task.report['recovery'][0])

    def test_mission_uses_its_own_mapped_participant(self):
        network, profile = run.mission_network(dict(dds_transport='udp_mapped', domain=94, pc_ip='192.0.2.1'))
        self.assertEqual(network, ['-p','192.0.2.1:30916-30917:30916-30917/udp'])
        self.assertEqual(profile, '/session/pc-dds-mission.xml')
        self.assertEqual(run.mission_network({}), (['--network','host'], '/session/pc-dds.xml'))

    def test_mapped_udp_preserves_native_transport_and_unique_ports(self):
        config = json.loads((run.HERE/'config.example.json').read_text())
        config['dds_transport'] = 'udp_mapped'
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'udp-01'
            run.prepare(out, config)
            ns = {'d':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
            for index, role in enumerate(['agent','sensors','clock','mission','observer']):
                tree = ET.parse(out/('pc-dds.xml' if role=='observer' else 'pc-dds-'+role+'.xml'))
                self.assertEqual([v.text for v in tree.findall('.//d:transport_descriptor/d:type',ns)],['UDPv4'])
                locators = tree.findall('.//d:metatrafficUnicastLocatorList/d:locator/d:udpv4',ns)
                self.assertEqual([v.find('d:address',ns).text for v in locators],[config['pc_ip'],'0.0.0.0'])
                self.assertEqual({int(v.find('d:port',ns).text) for v in locators},{7400+250*config['domain']+10+2*index})
            agent = ET.parse(out/'pc-dds-agent.xml').find('d:participant',ns)
            self.assertEqual(agent.attrib['profile_name'],'px4_participant')
            services=json.loads((out/'compose.json').read_text())['services']
            self.assertTrue(all('network_mode' not in v for v in services.values()))
            self.assertTrue(all(v.get('ports') for v in services.values()))

    def test_pc_participants_have_distinct_fixed_tcp_ports(self):
        config = json.loads((run.HERE/'config.example.json').read_text())
        config.update(dds_transport='tcp_board_listener', tcp_base_port=51700)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'tcp-01'
            run.prepare(out, config)
            ns = {'d':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
            ports = [int(ET.parse(path).find('.//d:listening_ports/d:port', ns).text)
                     for path in out.glob('pc-dds*.xml')]
            self.assertEqual(len(ports), 5)
            self.assertEqual(len(set(ports)), 5)
            self.assertTrue(all(1024 <= p < 51700 for p in ports))

    def test_experimental_tcp_never_starts_flight(self):
        with patch.object(run, 'board_action', side_effect=AssertionError('no device access')):
            with self.assertRaisesRegex(RuntimeError, 'flight disabled'):
                run.mission({'dds_transport':'tcp_board_listener'}, Path('.'), 'basic', None)

    def test_stop_fails_closed(self):
        safe = dict(fresh=True, armed=False, px4_armed=False, behavior_idle=True,
                    ego_state='WAITING', ego_goals=0, inputs_ready=True, interfaces_ready=True)
        self.assertTrue(capability_ready(safe))
        for key, value in [('fresh', False), ('armed', True), ('px4_armed', True),
                           ('behavior_idle', False), ('ego_state', 'RUNNING')]:
            with self.subTest(key=key):
                self.assertFalse(safe_to_stop(dict(safe, **{key:value})))
        self.assertFalse(safe_to_stop({}))
        self.assertTrue(safe_to_stop(dict(safe, ego_state='FAILED', stop_confirmed=True)))
        self.assertFalse(capability_ready(dict(safe, ego_goals=1)))

    def test_prepare_is_offline_and_preserves_vehicle(self):
        config = json.loads((run.HERE/'config.example.json').read_text())
        before = copy.deepcopy(config)
        with tempfile.TemporaryDirectory() as tmp, patch('subprocess.run', side_effect=AssertionError('must stay offline')):
            out = Path(tmp)/'offline-01'
            run.prepare(out, config)
            actual = json.loads((out/'settings.json').read_text())
            original = json.loads((run.PROJECT/'deployment/airsim/demos/ego_planner_v2/pc/settings.snapshot.json').read_text())
            self.assertEqual(list(actual['Vehicles']), ['Drone0'])
            self.assertEqual(actual['Vehicles']['Drone0'], original['Vehicles']['Drone0'])
            compose = json.loads((out/'compose.json').read_text())
            self.assertEqual(set(compose['services']), {'px4', 'clock'})
            self.assertTrue(all(s['pull_policy']=='never' for s in compose['services'].values()))
            ego = json.loads((out/'bundle/ego.json').read_text())
            self.assertEqual(ego['vehicles'], ['drone0'])
            self.assertEqual(config, before)
            with self.assertRaises(FileExistsError):
                run.prepare(out, config)

    def test_reject_unsafe_identity(self):
        for value in ('../x', '/tmp/x', '', 'a;stop', 'a b'):
            with self.assertRaises(ValueError):
                run_id(value)
        self.assertEqual(run_id('single-01'), 'single-01')

    def test_syntax(self):
        for file in run.HERE.glob('*.py'):
            ast.parse(file.read_text(), filename=str(file))

    def test_hdc_requires_remote_success_marker(self):
        with patch.object(run, 'hdc', return_value='EGO_EXIT=0\nEGO_EXIT=1\n'):
            with self.assertRaises(RuntimeError):
                run.board_shell({}, 'false')
        with patch.object(run, 'hdc', return_value='{"alive": false}\r\nEGO_EXIT=0\r\n'):
            self.assertEqual(json.loads(run.board_shell({}, 'status')), {'alive':False})

    def test_unready_mission_never_starts_container(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(run, 'board_action', return_value={'ready':False}), \
                patch('subprocess.run', side_effect=AssertionError('no motion/container allowed')):
            out = Path(tmp)
            (out/'workstation-sources.json').write_text('{}')
            with self.assertRaisesRegex(RuntimeError, 'not ready'):
                run.mission({}, out, 'basic', None)


if __name__ == '__main__':
    unittest.main()
