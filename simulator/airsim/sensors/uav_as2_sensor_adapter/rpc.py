"""Small bounded MessagePack-RPC client; no AirSim dependency above this layer."""
import socket
import msgpack


class Rpc:
    def __init__(self, host, port, timeout=3.0):
        self.address = (host, port)
        self.timeout = timeout
        self.sequence = 0

    def call(self, method, *args):
        self.sequence += 1
        request_id = self.sequence
        with socket.create_connection(self.address, self.timeout) as sock:
            sock.settimeout(self.timeout)
            sock.sendall(msgpack.packb([0, request_id, method, args], use_bin_type=True))
            unpacker = msgpack.Unpacker(raw=False, max_buffer_size=64 * 1024 * 1024)
            while True:
                chunk = sock.recv(65536)
                if not chunk:
                    raise ConnectionError('AirSim closed the RPC connection')
                unpacker.feed(chunk)
                for response in unpacker:
                    if response[0] == 1 and response[1] == request_id:
                        if response[2] is not None:
                            raise RuntimeError(str(response[2]))
                        return response[3]


class PersistentRpc(Rpc):
    """Serial, bounded RPC session; transport failures never replay a request."""
    def __init__(self, host, port, timeout=3.0):
        super().__init__(host, port, timeout)
        import threading
        self._lock = threading.Lock()
        self._socket = None
        self._unpacker = None

    def close(self):
        with self._lock:
            self._close()

    def _close(self):
        if self._socket is not None:
            self._socket.close()
        self._socket = None
        self._unpacker = None

    def call(self, method, *args):
        with self._lock:
            try:
                if self._socket is None:
                    self._socket = socket.create_connection(self.address, self.timeout)
                    self._socket.settimeout(self.timeout)
                    self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self._unpacker = msgpack.Unpacker(raw=False, max_buffer_size=64 * 1024 * 1024)
                self.sequence += 1
                request_id = self.sequence
                self._socket.sendall(msgpack.packb([0, request_id, method, args], use_bin_type=True))
                while True:
                    chunk = self._socket.recv(65536)
                    if not chunk:
                        raise ConnectionError('AirSim closed the RPC connection')
                    self._unpacker.feed(chunk)
                    for response in self._unpacker:
                        if response[0] == 1 and response[1] == request_id:
                            if response[2] is not None:
                                raise RuntimeError(str(response[2]))
                            return response[3]
            except BaseException:
                self._close()
                raise
