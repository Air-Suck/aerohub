import socket
import unittest
from unittest.mock import patch

import msgpack

from uav_as2_sensor_adapter.rpc import Rpc


class FakeSocket:
    def __init__(self, responses):
        self.responses = list(responses)
        self.sent = []
        self.closed = False
        self.shutdown_calls = 0
        self.timeout = None

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendall(self, payload):
        self.sent.append(msgpack.unpackb(payload, raw=False))

    def recv(self, _size):
        response = self.responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return response

    def shutdown(self, _how):
        self.shutdown_calls += 1

    def close(self):
        self.closed = True


def response(request_id, result=None, error=None):
    return msgpack.packb([1, request_id, error, result], use_bin_type=True)


class RpcTests(unittest.TestCase):
    def test_sequential_calls_reuse_one_connection(self):
        sock = FakeSocket([response(1, 'first'), response(2, 'second')])
        with patch('uav_as2_sensor_adapter.rpc.socket.create_connection',
                   return_value=sock) as connect:
            rpc = Rpc('127.0.0.1', 41471, 0.5)
            self.assertEqual(rpc.call('one', 1), 'first')
            self.assertEqual(rpc.call('two', 2), 'second')
            self.assertEqual(connect.call_count, 1)
            self.assertEqual([request[2] for request in sock.sent], ['one', 'two'])
            self.assertEqual(sock.timeout, 0.5)
            rpc.close()
        self.assertTrue(sock.closed)

    def test_failure_closes_without_replaying_and_next_call_reconnects(self):
        failed = FakeSocket([socket.timeout('slow')])
        recovered = FakeSocket([response(2, 'ok')])
        with patch('uav_as2_sensor_adapter.rpc.socket.create_connection',
                   side_effect=[failed, recovered]) as connect:
            rpc = Rpc('127.0.0.1', 41471)
            with self.assertRaises(socket.timeout):
                rpc.call('state-changing-call')
            self.assertTrue(failed.closed)
            self.assertEqual(len(failed.sent), 1)
            self.assertEqual(rpc.call('read-only-call'), 'ok')
            self.assertEqual(connect.call_count, 2)
            self.assertEqual(len(recovered.sent), 1)
            rpc.close()

    def test_remote_error_invalidates_connection(self):
        sock = FakeSocket([response(1, error='bad request')])
        with patch('uav_as2_sensor_adapter.rpc.socket.create_connection',
                   return_value=sock):
            rpc = Rpc('127.0.0.1', 41471)
            with self.assertRaisesRegex(RuntimeError, 'bad request'):
                rpc.call('bad')
        self.assertTrue(sock.closed)


if __name__ == '__main__':
    unittest.main()
