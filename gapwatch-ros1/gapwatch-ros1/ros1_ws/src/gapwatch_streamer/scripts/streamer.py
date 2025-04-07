#!/usr/bin/env python3

from __future__ import annotations

import socket
import struct

import numpy as np
import rospy
from gapwatch_streamer.msg import EMG


def _decode_fn(data: bytes) -> tuple[np.ndarray, int, int, int]:
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
    int
        Battery level.
    int
        Packet counter.
    int
        Timestamp in microseconds.
    """
    nSampEMG, nChEMG = 5, 16

    # ADC parameters
    vRef = 4
    gain = 6
    nBit = 24

    # Get data
    dataEMG = bytearray(data[:240])
    dataBat = bytearray(data[240:241])
    dataCounter = bytearray(data[241:243])
    dataTs = bytearray(data[244:252])

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
    battery = struct.unpack("<B", dataBat)[0]
    counter = struct.unpack(">H", dataCounter)[0]
    ts = struct.unpack("<Q", dataTs)[0]

    return emg, battery, counter, ts


class GAPWatch:
    """
    GAPWatch driver.

    Parameters
    ----------
    socket_port : int
        Socket port.
    packet_size : int, default=252
        Size of each packet read from the socket.

    Attributes
    ----------
    _socket_port : int
        Socket port.
    _packet_size : int
        Size of each packet read from the serial port.
    _sock : None or socket
        Server socket object.
    _conn : None or socket
        Client socket object.
    """

    def __init__(self, socket_port: int, packet_size: int = 252) -> None:
        self._socket_port = socket_port
        self._packet_size = packet_size
        self._sock = None
        self._conn = None

    def start(self) -> None:
        """Start transmission."""
        # Open socket and wait for connections
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("", self._socket_port))
        self._sock.listen()

        rospy.loginfo(f"Waiting for TCP connection on port {self._socket_port}.")

        self._sock.settimeout(1.0)

        while not rospy.is_shutdown():
            try:
                self._conn, (addr, _) = self._sock.accept()

                rospy.loginfo(f"New TCP connection with {addr}:{self._socket_port}")

                # Start command sequence
                self._conn.sendall(b"=")
                break
            except socket.timeout:
                pass
            except rospy.ROSInterruptException:
                break

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
        if self._sock is None or self._conn is None:
            return

        # Stop command
        self._conn.sendall(b":")

        # Close socket
        self._conn.shutdown(socket.SHUT_RDWR)
        self._conn.close()
        self._sock.close()


def main():
    # Initialize the ROS node
    rospy.init_node("streamer", anonymous=True)

    # GAPWatch driver
    socket_port = rospy.get_param("~socket_port", 3333)
    gapwatch = GAPWatch(socket_port)
    pub = rospy.Publisher("emg", EMG, queue_size=10)
    rate = rospy.Rate(400)  # 400 Hz

    # Close socket on exit
    rospy.on_shutdown(gapwatch.stop)

    # Start acquiring
    gapwatch.start()
    while not rospy.is_shutdown():
        emg, battery, counter, ts = gapwatch.get_emg()

        msg = EMG()
        msg.emg = emg.flatten().tolist()
        msg.battery = battery
        msg.counter = counter
        msg.ts = ts
        pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
