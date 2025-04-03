"""
ROS2 wrapper for GAPWatch.


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
            logger=self.get_logger(),
        )
        self._gapwatch.start()
        self.timer = self.create_timer(0.0025, self.publish_emg)

    def publish_emg(self) -> None:
        emg, battery, counter, ts = self._gapwatch.get_emg()
        emg_msg = EMG()
        emg_msg.emg = emg.flatten()
        emg_msg.battery = battery.flatten()
        emg_msg.counter = counter.flatten()
        emg_msg.ts = ts.flatten()

        self._emg_publisher.publish(emg_msg)

    def __del__(self) -> None:
        self._gapwatch.stop()


def main():
    rclpy.init()

    try:
        streamer = Streamer()

        # while rclpy.ok():
        #    streamer.publish_emg()
        rclpy.spin(streamer)
    except KeyboardInterrupt:
        print("Manual shutdown.")
    except Exception as e:
        print(f"Exception {e} occurred.")
    finally:
        # Shutdown
        if rclpy.ok():
            streamer.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
