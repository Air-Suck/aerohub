"""Application-local AirSim TCP startup guard; leaves all MAVLink payloads unchanged."""
import json
import os
from pathlib import Path
import select
import socket
import socketserver
import subprocess
import threading
import time


class StartupRelay(socketserver.ThreadingTCPServer):
    allow_reuse_address=True
    daemon_threads=True

    def __init__(self,listen,upstream,delay=.1):
        self.upstream=upstream;self.delay=delay;self.stopping=threading.Event()
        class Handler(socketserver.BaseRequestHandler):
            def handle(handler):
                try:
                    handler.request.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    with socket.create_connection(upstream,timeout=5) as peer:
                        peer.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                        # AirSim acceptTcp starts its reader before registering the
                        # heartbeat callback. Buffer the first bytes until registration.
                        if self.stopping.wait(delay):return
                        while not self.stopping.is_set():
                            ready,_,_=select.select([handler.request,peer],[],[],.2)
                            for source in ready:
                                data=source.recv(65536)
                                if not data:return
                                (peer if source is handler.request else handler.request).sendall(data)
                except (OSError,ValueError):return
        super().__init__(listen,Handler)
        self.thread=threading.Thread(target=self.serve_forever,kwargs={'poll_interval':.1},daemon=True)
        self.thread.start()

    def close(self):
        self.stopping.set();self.shutdown();self.server_close();self.thread.join(timeout=2)


def launch_vehicles(image,app,output,prefix,dds,delay=.1,sensors=False):
    settings=json.loads((app/'settings.json').read_text())
    count=len(settings['Vehicles'])
    relays=[]
    try:
        for index in range(count):
            original=int(settings['Vehicles'][f'Drone{index}']['TcpPort'])
            relays.append(StartupRelay(('127.0.0.1',4780+index),('127.0.0.1',original),delay))
            settings['Vehicles'][f'Drone{index}']['TcpPort']=4780+index
        path=output/'px4-settings.json';path.write_text(json.dumps(settings,indent=2)+'\n')
        for index in range(count):
            subprocess.run(['docker','run','--rm','--detach','--init','--network','host','--name',prefix+f'vehicle{index}',
                '-e','ROS_DOMAIN_ID=91','-e',f'VEHICLE_NAME=Drone{index}','-e',f'VEHICLE_NAMESPACE=drone{index}',
                '-e',f'PX4_INSTANCE={40+index}','-e',f'PX4_SYSTEM_ID={41+index}','-e',f'XRCE_AGENT_PORT={8901+index}',
                '-e','AIRSIM_RPC_PORT=42471','-e','FLIGHT_BOARD_SENSORS='+('1' if sensors else '0'),'-e','FLIGHT_BOARD_CAMERAS=0',
                '-v',str(path)+':/config/settings.json:ro','-v',str(dds)+':/opt/fastdds_udp.xml:ro',image,'vehicle'],check=True,stdout=subprocess.DEVNULL)
        startup_timeout=float(os.environ.get('PX4_STARTUP_TIMEOUT','25'))
        if not 1 <= startup_timeout <= 300:
            raise ValueError('PX4_STARTUP_TIMEOUT must be in [1, 300] seconds')
        started=time.monotonic()
        deadline=started+startup_timeout
        while True:
            logs={i:subprocess.check_output(['docker','logs',prefix+f'vehicle{i}'],stderr=subprocess.STDOUT,text=True) for i in range(count)}
            ready=[i for i,text in logs.items() if 'Ready for takeoff!' in text]
            if len(ready)==count:
                (output/'px4-startup.json').write_text(json.dumps({
                    'ready_vehicles':ready,'initial_tcp_buffer_seconds':delay,'tcp_nodelay':True,
                    'relay_ports':list(range(4780,4780+count)),
                    'startup_timeout_seconds':startup_timeout,
                    'startup_wall_seconds':time.monotonic()-started},indent=2)+'\n')
                return relays
            if time.monotonic()>deadline:raise RuntimeError('PX4 startup handshake incomplete; ready vehicles='+str(ready))
            time.sleep(.25)
    except BaseException:
        for relay in relays:relay.close()
        raise
