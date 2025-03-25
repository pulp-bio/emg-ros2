"""
This module contains the BioGAP-ROS2 interface for sEMG.


Copyright 2023 Mattia Orlandi, Pierangelo Maria Rapa

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

import numpy as np

packetSize: int = 330
"""Number of bytes in each package."""

startSeq: list[bytes] = []
"""Sequence of commands to start the device."""

stopSeq: list[bytes] = []
"""Sequence of commands to stop the device."""

sigInfo: dict = {
    "emg": {"fs": 2000, "nCh": 16},
    "battery": {"fs": 400, "nCh": 1},
    "counter": {"fs": 400, "nCh": 1},
    "ts": {"fs": 400, "nCh": 1},
}
"""Dictionary containing the signals information."""


def decodeFn(data: bytes) -> dict[str, np.ndarray]:
    """
    Function to decode the binary data received from the device into signals.

    Parameters
    ----------
    data : bytes
        A packet of bytes.

    Returns
    -------
    dict of (str: ndarray)
        Dictionary containing the signal data packets, each with shape (nSamp, nCh);
        the keys must match with those of the "sigInfo" dictionary.
    """
    emg = np.frombuffer(data[:320], dtype=np.float32).reshape(5, 16)
    battery = np.frombuffer(data[320:321], dtype=np.uint8).reshape(1, 1)
    counter = np.frombuffer(data[321:322], dtype=np.uint8).reshape(1, 1)
    ts = np.frombuffer(data[322:], dtype=np.uint64).reshape(1, 1)

    return {"emg": emg, "battery": battery, "counter": counter, "ts": ts}
