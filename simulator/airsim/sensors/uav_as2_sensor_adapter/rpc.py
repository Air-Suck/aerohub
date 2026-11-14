"""Small bounded MessagePack-RPC client; no AirSim dependency above this layer."""
import socket
import msgpack


class Rpc:
    def __init__(self, host, port, timeout=3.0):
        self.address = (host, port)
        self.timeout = timeout
        self.sequence = 0
        self.socket = None
        self.unpacker = None

    def connect(self):
        if self.socket is not None:
            return
        sock = socket.create_connection(self.address, self.timeout)
        try:
            sock.settimeout(self.timeout)
        except Exception:
            sock.close()
            raise
        self.socket = sock
        self.unpacker = msgpack.Unpacker(
            raw=False, max_buffer_size=64 * 1024 * 1024)

    def close(self):
        sock, self.socket = self.socket, None
        self.unpacker = None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        sock.close()

    def call(self, method, *args):
        self.sequence += 1
        request_id = self.sequence
        try:
            self.connect()
            self.socket.sendall(
                msgpack.packb([0, request_id, method, args], use_bin_type=True))
            while True:
                chunk = self.socket.recv(65536)
                if not chunk:
                    raise ConnectionError('AirSim closed the RPC connection')
                self.unpacker.feed(chunk)
                for response in self.unpacker:
                    if response[0] != 1:
                        continue
                    if response[1] != request_id:
                        raise RuntimeError('unexpected AirSim RPC response id')
                    if response[2] is not None:
                        raise RuntimeError(str(response[2]))
                    return response[3]
        except Exception:
            # Do not replay a potentially state-changing call. The next call
            # opens a fresh connection after the caller handles this failure.
            self.close()
            raise
