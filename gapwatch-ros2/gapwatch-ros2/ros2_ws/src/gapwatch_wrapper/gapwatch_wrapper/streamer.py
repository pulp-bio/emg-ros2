"""ROS2 wrapper for GAPWatch.


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
from gapwatch_messages.msg import EMG
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

try:
    from gapwatch import GAPWatch
except ModuleNotFoundError:
    import sys

    sys.path.insert(0, "..")

    from gapwatch import GAPWatch


class Streamer(Node):

    def __init__(self) -> None:
        super().__init__("streamer")
        self._emg_publisher = self.create_publisher(EMG, "emg", 100)

        # Serial port
        self.declare_parameter(
            "socket_port",
            3333,
            ParameterDescriptor(description="Socket port to communicate with GAPWatch"),
        )
        self._gapwatch = GAPWatch(
            socket_port=self.get_parameter("socket_port")
            .get_parameter_value()
            .integer_value,
        )
        self._gapwatch.start()
        self.get_logger().info("Streamer started.")

    def publish_emg(self) -> None:
        data = self._gapwatch.get_emg()
        emg = EMG()
        emg.data = data.flatten()

        self._emg_publisher.publish(emg)

    def __del__(self) -> None:
        self._gapwatch.stop()
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
