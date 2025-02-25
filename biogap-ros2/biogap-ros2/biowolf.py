"""BioWolf driver.


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

import struct
import time

import numpy as np
import serial


def _decode_fn(data: bytes) -> np.ndarray:
    """Function to decode the binary data received from BioWolf into a single sEMG signal.

    Parameters
    ----------
    data : bytes
        A packet of bytes.

    Returns
    -------
    ndarray
        EMG packet with shape (nSamp, nCh).
    """
    n_samp = 5
    n_ch = 16

    # ADC parameters
    v_ref = 2.5
    gain = 6.0
    n_bit = 24

    data_tmp = bytearray(
        [x for i, x in enumerate(data) if i not in (0, 1, 242)]
    )  # discard header and footer

    # Convert 24-bit to 32-bit integer
    pos = 0
    for _ in range(len(data_tmp) // 3):
        prefix = 255 if data_tmp[pos] > 127 else 0
        data_tmp.insert(pos, prefix)
        pos += 4
    emg = np.asarray(struct.unpack(f">{n_samp * n_ch}i", data_tmp), dtype="int32")

    # Reshape and convert ADC readings to uV
    emg = emg.reshape(n_samp, n_ch)
    emg = emg * (v_ref / gain / 2**n_bit)  # V
    emg *= 1_000_000  # uV
    emg = emg.astype("float32")

    return emg


class BioWolf:
    """BioWolf driver.

    Parameters
    ----------
    serial_port : str
        String representing the serial port.
    packet_size : int, default=243
        Size of each packet read from the serial port.
    baud_rate : int, default=4000000
        Baud rate.

    Attributes
    ----------
    _serial_port : str
        String representing the serial port.
    _packet_size : int
        Size of each packet read from the serial port.
    _baud_rate : int
        Baud rate.
    _ser : Serial or None
        Serial port object (initialized to None).
    """

    def __init__(
        self, serial_port: str, packet_size: int = 243, baud_rate: int = 4000000
    ) -> None:
        self._serial_port = serial_port
        self._packet_size = packet_size
        self._baud_rate = baud_rate

        self._ser = None

    def start(self) -> None:
        """Start transmission."""
        # Open serial port
        self._ser = serial.Serial(self._serial_port, self._baud_rate, timeout=5)

        # Start command sequence
        self._ser.write(b"=")

    def get_emg(self) -> np.ndarray:
        """Read a packet of EMG data.

        Returns
        -------
        ndarray
            Packet of EMG data.
        """
        assert self._ser is not None, "Attempting to read from a closed serial port."

        # Read data from serial port and decode it
        data = self._ser.read(self._packet_size)
        emg = _decode_fn(data)

        return emg

    def stop(self) -> None:
        """Stop transmission."""
        assert self._ser is not None, "Attempting to close a closed serial port."

        # Stop command
        self._ser.write(b":")

        # Close serial port
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        time.sleep(0.2)
        self._ser.close()
