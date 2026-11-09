import socket,socketserver,threading,time,unittest
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

if __name__=='__main__':unittest.main()
