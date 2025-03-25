"""
GAPWatch ROS2 driver.


Copyright 2024 Mattia Orlandi, Pierangelo Maria Rapa

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

from __future__ import annotations

import socket
import struct

import numpy as np


BUFF_SIZE = 1


def _decode_fn(data: bytes) -> tuple[np.ndarray, ...]:
    """
    Function to decode the binary data received from GAPWatch into a single sEMG signal.

    Parameters
    ----------
    data : bytes
        A packet of bytes.

    Returns
    -------
    ndarray
        EMG packet with shape (nSamp, nCh).
    """
    nSampEMG, nChEMG = 5 * BUFF_SIZE, 16
    nSampBat = nSampCounter = nSampTs = 1 * BUFF_SIZE

    # ADC parameters
    vRef = 4
    gain = 6
    nBit = 24

    dataEMG = bytearray()
    dataBat = bytearray()
    dataCounter = bytearray()
    dataTs = bytearray()
    for i in range(BUFF_SIZE):
        dataEMG.extend(bytearray(data[i * 252 : i * 252 + 240]))
        dataBat.extend(bytearray(data[i * 252 + 240 : i * 252 + 241]))
        dataCounter.extend(bytearray(data[i * 252 + 241 : i * 252 + 243]))
        dataTs.extend(bytearray(data[i * 252 + 244 : i * 252 + 252]))

    # Convert 24-bit to 32-bit integer
    pos = 0
    for _ in range(len(dataEMG) // 3):
        prefix = 255 if dataEMG[pos] > 127 else 0
        dataEMG.insert(pos, prefix)
        pos += 4
    emgADC = np.asarray(
        struct.unpack(f">{nSampEMG * nChEMG}i", dataEMG), dtype=np.int32
    ).reshape(nSampEMG, nChEMG)

    # ADC readings to mV
    emg = emgADC * vRef / (gain * (2 ** (nBit - 1) - 1))  # V
    emg *= 1_000  # mV
    emg = emg.astype(np.float32)

    # Read battery and packet counter
    battery = np.asarray(
        struct.unpack(f"<{nSampBat}B", dataBat), dtype=np.uint8
    ).reshape(nSampBat, 1)
    counter = np.asarray(
        struct.unpack(f">{nSampCounter}H", dataCounter), dtype=np.uint8
    ).reshape(nSampCounter, 1)
    ts = np.asarray(
        struct.unpack(f"<{nSampTs}Q", dataTs), dtype=np.uint64
    ).reshape(nSampTs, 1)

    return emg, battery, counter, ts


class GAPWatch:
    """
    GAPWatch driver.

    Parameters
    ----------
    socket_port : int
        Socket port.
    packet_size : int, default=252 * BUFF_SIZE
        Size of each packet read from the socket.

    Attributes
    ----------
    _packet_size : int
        Size of each packet read from the serial port.
    """

    def __init__(self, socket_port: int, packet_size: int = 252 * BUFF_SIZE, logger = None) -> None:
        self._socket_port = socket_port
        self._packet_size = packet_size
        self._logger = logger
        self._sock = None
        self._conn = None

    def start(self) -> None:
        """Start transmission."""
        # Open socket and wait for connections
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("", self._socket_port))
        self._sock.listen()
        self._conn, _ = self._sock.accept()

        # Start command sequence
        self._conn.sendall(b"=")

    def get_emg(self) -> tuple[np.ndarray, ...]:
        """Read a packet of EMG data.

        Returns
        -------
        ndarray
            Packet of EMG data.
        """
        assert (
            self._sock is not None and self._conn is not None
        ), "Attempting to read from a closed socket."

        # Read data from socket and decode it
        data = bytearray(self._packet_size)
        pos = 0
        while pos < self._packet_size:
            nRead = self._conn.recv_into(memoryview(data)[pos:])
            pos += nRead

        return _decode_fn(data)

    def stop(self) -> None:
        """Stop transmission."""
        assert (
            self._sock is not None and self._conn is not None
        ), "Attempting to close a closed socket."

        # Stop command
        self._conn.sendall(b":")

        # Close socket
        self._conn.shutdown(socket.SHUT_RDWR)
        self._conn.close()
        self._sock.close()
