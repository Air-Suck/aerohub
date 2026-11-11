import json,socket,socketserver,sys,tempfile,threading,time,unittest
from pathlib import Path
from unittest import mock
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
import sim_startup
from sim_startup import StartupRelay


class FirstPacketTest(unittest.TestCase):
    def test_first_packet_waits_for_callback_and_payloads_remain_identical(self):
        observations=[]
        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                accepted=time.monotonic()
                for index in range(2):
                    value=self.request.recv(64);observations.append((index,time.monotonic()-accepted,value));self.request.sendall(value)
        upstream=socketserver.TCPServer(('127.0.0.1',0),Handler)
        worker=threading.Thread(target=upstream.serve_forever,kwargs={'poll_interval':.05},daemon=True);worker.start()
        relay=StartupRelay(('127.0.0.1',0),upstream.server_address,.1)
        try:
            with socket.create_connection(relay.server_address,timeout=2) as client:
                for value in (b'HEARTBEAT',b'UNCHANGED-MAVLINK-PAYLOAD'):
                    client.sendall(value);self.assertEqual(client.recv(64),value)
            self.assertGreaterEqual(observations[0][1],.08)
            self.assertLess(observations[1][1]-observations[0][1],.1)
        finally:relay.close();upstream.shutdown();upstream.server_close();worker.join(timeout=2)


class SmallPacketLatencyTest(unittest.TestCase):
    @unittest.skipUnless(hasattr(socket,'TCP_QUICKACK'),'Linux delayed-ACK regression')
    def test_forwarding_does_not_add_nagle_delayed_ack_queue(self):
        import struct
        ages=[]
        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                self.request.setsockopt(socket.IPPROTO_TCP,socket.TCP_QUICKACK,0)
                data=b''
                while len(ages)<100:
                    chunk=self.request.recv(65536)
                    if not chunk:return
                    data+=chunk
                    while len(data)>=12:
                        index,stamp=struct.unpack('!Id',data[:12]);data=data[12:]
                        if index>=40:ages.append(time.monotonic()-stamp)
                    self.request.setsockopt(socket.IPPROTO_TCP,socket.TCP_QUICKACK,0)
        upstream=socketserver.TCPServer(('127.0.0.1',0),Handler)
        worker=threading.Thread(target=upstream.serve_forever,kwargs={'poll_interval':.05},daemon=True);worker.start()
        relay=StartupRelay(('127.0.0.1',0),upstream.server_address,.1)
        try:
            with socket.create_connection(relay.server_address,timeout=3) as client:
                client.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                for index in range(140):
                    client.sendall(struct.pack('!Id',index,time.monotonic()));time.sleep(.005)
                deadline=time.monotonic()+2
                while len(ages)<100 and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(len(ages),100)
            self.assertLess(max(ages),.030,'relay adds delayed-ACK/Nagle latency to the HIL stream')
        finally:relay.close();upstream.shutdown();upstream.server_close();worker.join(timeout=2)

class LaunchVehiclesDefaultsTest(unittest.TestCase):
    """Existing callers pass neither settings_path nor vehicles; their commands must stay unchanged."""
    ROOT=Path(__file__).resolve().parents[5]
    SCENES=('catora_open_field','swarm_open_field')

    def launch(self,app,output,**kwargs):
        runs=[];relays=[]
        class Relay:
            def __init__(self,listen,upstream,delay):relays.append((listen,upstream,delay))
            def close(self):pass
        def logs(command,**_):return 'Ready for takeoff!'
        with mock.patch.object(sim_startup,'StartupRelay',Relay),\
             mock.patch.object(sim_startup.subprocess,'run',side_effect=lambda command,**_:runs.append(command)),\
             mock.patch.object(sim_startup.subprocess,'check_output',side_effect=logs):
            sim_startup.launch_vehicles('board:image',app,output,'pre-',Path('/dds.xml'),**kwargs)
        return runs,relays

    def test_default_settings_and_vehicle_identity_are_unchanged(self):
        for scene in self.SCENES:
            with self.subTest(scene=scene),tempfile.TemporaryDirectory() as temporary:
                app=self.ROOT/'simulator/airsim/scenarios'/scene;output=Path(temporary)
                original=json.loads((app/'settings.json').read_text())
                runs,relays=self.launch(app,output)
                count=len(original['Vehicles'])
                expected_settings=json.loads((app/'settings.json').read_text())
                for index in range(count):expected_settings['Vehicles'][f'Drone{index}']['TcpPort']=4780+index
                self.assertEqual(expected_settings,json.loads((output/'px4-settings.json').read_text()))
                self.assertEqual([(('127.0.0.1',4780+i),('127.0.0.1',int(original['Vehicles'][f'Drone{i}']['TcpPort'])),.1) for i in range(count)],relays)
                self.assertEqual([['docker','run','--rm','--detach','--init','--network','host','--name',f'pre-vehicle{i}',
                    '-e','ROS_DOMAIN_ID=91','-e',f'VEHICLE_NAME=Drone{i}','-e',f'VEHICLE_NAMESPACE=drone{i}',
                    '-e',f'PX4_INSTANCE={40+i}','-e',f'PX4_SYSTEM_ID={41+i}','-e',f'XRCE_AGENT_PORT={8901+i}',
                    '-e','AIRSIM_RPC_PORT=42471','-e','FLIGHT_BOARD_SENSORS=0','-e','FLIGHT_BOARD_CAMERAS=0',
                    '-v',str(output/'px4-settings.json')+':/config/settings.json:ro','-v','/dds.xml:/opt/fastdds_udp.xml:ro','board:image','vehicle'] for i in range(count)],runs)
                self.assertEqual(list(range(count)),json.loads((output/'px4-startup.json').read_text())['ready_vehicles'])

    def test_explicit_settings_path_and_vehicles_override_identity(self):
        with tempfile.TemporaryDirectory() as temporary:
            output=Path(temporary);settings=output/'generated.json'
            settings.write_text(json.dumps({'Vehicles':{'Drone0':{'TcpPort':5000},'Drone1':{'TcpPort':5001}}}))
            vehicles=[{'name':f'drone{i}','airsim_name':f'Drone{i}','px4_instance':60+i,'px4_system_id':61+i,'xrce_agent_port':9000+i} for i in range(2)]
            runs,relays=self.launch(output/'missing-app',output,settings_path=settings,vehicles=vehicles)
            self.assertEqual([5000,5001],[upstream[1] for _,upstream,_ in relays])
            self.assertIn('PX4_INSTANCE=61',runs[1]);self.assertIn('PX4_SYSTEM_ID=62',runs[1]);self.assertIn('XRCE_AGENT_PORT=9001',runs[1])
            with self.assertRaisesRegex(ValueError,'must match'):
                self.launch(output,output,settings_path=settings,vehicles=list(reversed(vehicles)))

if __name__=='__main__':unittest.main()
