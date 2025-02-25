"""ROS2 wrapper for BioGAP.


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

import rclpy
from biogap_messages.msg import EMG
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

try:
    from biogap import BioGAP
except ModuleNotFoundError:
    import sys

    sys.path.insert(0, "..")

    from biogap import BioGAP


class Streamer(Node):

    def __init__(self) -> None:
        super().__init__("streamer")
        self._emg_publisher = self.create_publisher(EMG, "emg", 100)

        # Serial port
        self.declare_parameter(
            "serial_port",
            "",
            ParameterDescriptor(description="Serial port to communicate with BioGAP"),
        )
        self.declare_parameter(
            "baud_rate",
            256000,
            ParameterDescriptor(description="Baud rate for the serial communication"),
        )
        self.declare_parameter(
            "gain",
            6,
            ParameterDescriptor(description="PGA gain of BioGAP"),
        )
        self._biogap = BioGAP(
            serial_port=self.get_parameter("serial_port")
            .get_parameter_value()
            .string_value,
            baud_rate=self.get_parameter("baud_rate")
            .get_parameter_value()
            .integer_value,
            gain=self.get_parameter("gain").get_parameter_value().integer_value,
        )
        self._biogap.start()
        self.get_logger().info("Streamer started.")

    def publish_emg(self) -> None:
        data = self._biogap.get_emg()
        emg = EMG()
        emg.data = data.flatten()

        self._emg_publisher.publish(emg)

    def __del__(self) -> None:
        self._biogap.stop()
        self.get_logger().info("Streamer stopped.")


def main():
    # Initialization
    rclpy.init()
    streamer = Streamer()

    while rclpy.ok():
        streamer.publish_emg()

    # Shutdown
    streamer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
