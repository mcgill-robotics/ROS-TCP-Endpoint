#  Copyright 2020 Unity Technologies
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""
A small stand-in for the Unity side of the bridge (ROS-TCP-Connector).

It speaks the same wire format as ROSConnection.cs:
    int32 (little endian) length of destination, destination bytes,
    int32 (little endian) length of payload, payload bytes.
Destinations starting with "__" are sys commands with a JSON payload.
An empty destination with an empty payload is a keepalive.
"""

import json
import queue
import socket
import struct
import threading
import time


class FakeUnity:
    def __init__(self, host="127.0.0.1", port=10000, connect_timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=connect_timeout)
        self.sock.settimeout(None)
        self.messages = queue.Queue()
        self.keepalives_received = 0
        self.closed_by_peer = threading.Event()
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    # ---- sending ----

    def send_raw(self, destination, payload):
        # Unity's MessageSerializer.Write(string) adds a null terminator in ROS2 mode.
        dest = destination.encode("utf-8") + b"\x00" if destination else b""
        self.sock.sendall(
            struct.pack("<I", len(dest)) + dest + struct.pack("<I", len(payload)) + payload
        )

    def syscommand(self, command, **params):
        # WriteUnaligned(json) in ROS2 mode also adds a null terminator,
        # which the endpoint strips with [:-1].
        self.send_raw(command, json.dumps(params).encode("utf-8") + b"\x00")

    def keepalive(self):
        self.sock.sendall(b"\x00" * 8)

    def close(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()

    # ---- receiving ----

    def _recvall(self, size):
        buf = b""
        while len(buf) < size:
            chunk = self.sock.recv(size - len(buf))
            if not chunk:
                raise IOError("closed")
            buf += chunk
        return buf

    def _read_loop(self):
        try:
            while True:
                (dest_len,) = struct.unpack("<I", self._recvall(4))
                dest = self._recvall(dest_len).decode("utf-8").rstrip("\x00")
                (msg_len,) = struct.unpack("<I", self._recvall(4))
                payload = self._recvall(msg_len)
                if dest == "":
                    self.keepalives_received += 1
                else:
                    self.messages.put((dest, payload))
        except (IOError, OSError):
            self.closed_by_peer.set()

    def wait_for(self, destination, timeout=5.0):
        """Return the payload of the next message for destination, skipping others."""
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            try:
                dest, payload = self.messages.get(timeout=remaining)
            except queue.Empty:
                return None
            if dest == destination:
                return payload
