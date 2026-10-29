"""Keep TESSE's metadata reply endpoint listening between requests.

TESSE opens a TCP connection back to this endpoint. Repeatedly closing it
between queries can stall a Unity physics frame on a refused callback connection.
"""
import socket
import struct
import math
import time
from tesse.msgs import MetadataRequest


class PersistentMetadataClient:
    def __init__(self, env, host, port, timeout=0.2):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Metadata timeout must be finite and positive')
        self.env = env
        self.timeout = timeout
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(16)
        self.listener.settimeout(timeout)

    @staticmethod
    def receive_exact(connection, length, deadline=None):
        parts = []
        while length:
            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise socket.timeout('TESSE metadata request deadline exceeded')
                connection.settimeout(remaining)
            part = connection.recv(length)
            if not part:
                raise ConnectionError('TESSE closed an incomplete metadata reply')
            parts.append(part)
            length -= len(part)
        return b''.join(parts)

    def request(self):
        # One wall-clock budget covers accept, header and body. A peer sending
        # tiny fragments must not postpone the controller's watchdog forever.
        deadline = time.monotonic() + self.timeout
        self.env.send(MetadataRequest())
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise socket.timeout('TESSE metadata request deadline exceeded')
        self.listener.settimeout(remaining)
        connection, _ = self.listener.accept()
        with connection:
            if self.receive_exact(connection, 4, deadline) != b'meta':
                raise ValueError('Unexpected metadata reply tag')
            length = struct.unpack('I', self.receive_exact(connection, 4, deadline))[0]
            if not 0 < length <= 1024 * 1024:
                raise ValueError('Invalid metadata reply length')
            return self.receive_exact(connection, length, deadline)

    def close(self):
        self.listener.close()
