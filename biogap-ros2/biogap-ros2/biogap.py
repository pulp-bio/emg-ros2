"""
BioGAP driver.


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


def _decode_fn(data: bytes, gain: int) -> np.ndarray:
    """
    Function to decode the binary data received from BioGAP into a single sEMG signal.

    Parameters
    ----------
    data : bytes
        A packet of bytes.
    gain : int
        PGA gain for the conversion to volts.

    Returns
    -------
    ndarray
        EMG packet with shape (nSamp, nCh).
    """
    n_samp, n_ch = 7, 8

    # ADC parameters
    v_ref = 2.5
    n_bit = 24

    data_tmp = bytearray(
        data[2:26]
        + data[34:58]
        + data[66:90]
        + data[98:122]
        + data[130:154]
        + data[162:186]
        + data[194:218]
    )
    # Convert 24-bit to 32-bit integer
    pos = 0
    for _ in range(len(data_tmp) // 3):
        prefix = 255 if data_tmp[pos] > 127 else 0
        data_tmp.insert(pos, prefix)
        pos += 4
    emg_adc = np.asarray(
        struct.unpack(f">{n_samp * n_ch}i", data_tmp), dtype=np.int32
    ).reshape(n_samp, n_ch)

    # Reshape and convert ADC readings to mV
    emg = (emg_adc * v_ref / (gain * (2 ** (n_bit - 1) - 1))).astype("float32")  # V
    emg *= 1_000  # mV

    return emg


class BioGAP:
    """
    BioGAP driver.

    Parameters
    ----------
    serial_port : str
        String representing the serial port.
    baud_rate : int, default=256000
        Baud rate.
    gain : int, default=6
        PGA gain for the conversion to volts (available values: 1, 2, 4, 6, 8, 12).
    packet_size : int, default=224
        Size of each packet read from the serial port.

    Attributes
    ----------
    _serial_port : str
        String representing the serial port.
    _baud_rate : int
        Baud rate.
    _packet_size : int
        Size of each packet read from the serial port.
    _ser : Serial or None
        Serial port object (initialized to None).
    """

    def __init__(
        self,
        serial_port: str,
        baud_rate: int = 256000,
        gain: int = 6,
        packet_size: int = 234,
    ) -> None:
        assert gain in (
            1,
            2,
            4,
            6,
            8,
            12,
        ), f'The "gain" parameter can be either 1, 2, 4, 6, 8, 12; {gain} was passed.'
        self._serial_port = serial_port
        self._baud_rate = baud_rate
        self._gain = gain
        self._packet_size = packet_size

        self._ser = None

    def start(self) -> None:
        """Start transmission."""
        # Open serial port
        self._ser = serial.Serial(self._serial_port, self._baud_rate, timeout=5)

        # Start command sequence
        gain_cmd_map = {
            1: 16,
            2: 32,
            4: 64,
            6: 0,
            8: 80,
            12: 96,
        }
        self._ser.write(bytes([20, 1, 50]))
        time.sleep(0.2)
        self._ser.write((18).to_bytes(length=1, byteorder="big"))
        params = [6, 0, 1, 4, gain_cmd_map[self._gain], 13, 10]
        self._ser.write(bytes(params))

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
        emg = _decode_fn(data, self._gain)

        return emg

    def stop(self) -> None:
        """Stop transmission."""
        assert self._ser is not None, "Attempting to close a closed serial port."

        # Stop command
        self._ser.write((19).to_bytes(length=1, byteorder="big"))

        # Close serial port
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        time.sleep(0.2)
        self._ser.close()
