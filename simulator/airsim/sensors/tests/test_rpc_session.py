import socket
import sys
from pathlib import Path
import threading
import unittest
import msgpack

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from uav_as2_sensor_adapter.rpc import PersistentRpc


class RpcSessionTest(unittest.TestCase):
    def test_multiple_calls_share_one_real_tcp_session(self):
        server = socket.socket()
        server.bind(('127.0.0.1', 0))
        server.listen(1)
        server.settimeout(3)
        received = []
        def serve():
            connection, _ = server.accept()
            with connection:
                unpacker = msgpack.Unpacker(raw=False)
                while len(received) < 3:
                    data = connection.recv(65536)
                    if not data:
                        return
                    unpacker.feed(data)
                    for _, sequence, method, args in unpacker:
                        received.append(method)
                        connection.sendall(msgpack.packb([1, sequence, None, args[0]], use_bin_type=True))
        worker = threading.Thread(target=serve)
        worker.start()
        rpc = PersistentRpc('127.0.0.1', server.getsockname()[1], timeout=2)
        try:
            for i in range(3):
                self.assertEqual(i, rpc.call('echo', i))
            self.assertEqual(['echo'] * 3, received)
        finally:
            rpc.close()
            worker.join(timeout=3)
            server.close()
        self.assertFalse(worker.is_alive())

    def test_disconnected_request_is_not_replayed(self):
        server = socket.socket()
        server.bind(('127.0.0.1', 0))
        server.listen(2)
        accepted = []
        def serve():
            connection, _ = server.accept()
            accepted.append(1)
            with connection:
                connection.recv(65536)
        worker = threading.Thread(target=serve)
        worker.start()
        rpc = PersistentRpc('127.0.0.1', server.getsockname()[1], timeout=1)
        try:
            with self.assertRaises((ConnectionError, OSError)):
                rpc.call('must_not_replay')
            self.assertEqual([1], accepted)
            self.assertIsNone(rpc._socket)
        finally:
            rpc.close()
            worker.join(timeout=2)
            server.close()
