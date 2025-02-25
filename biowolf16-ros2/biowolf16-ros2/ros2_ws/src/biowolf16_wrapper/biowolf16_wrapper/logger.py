"""
Object that logs EMG data to a pipe.


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

import os

import rclpy
from biowolf16_messages.msg import EMG
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class Logger(Node):

    def __init__(self) -> None:

        super().__init__("logger")
        self._emg_subscription = self.create_subscription(
            EMG, "emg", self._emg_callback, 100
        )
        self._emg_subscription

        # Open FIFO
        self.declare_parameter(
            "fifo_path",
            "/root/shared/fifo",
            ParameterDescriptor(description="Path to the FIFO"),
        )
        self._path = self.get_parameter("fifo_path").get_parameter_value().string_value
        if os.path.exists(self._path):
            os.remove(self._path)
        os.mkfifo(self._path)
        self._fifo = open(self._path, "wb")

        self.get_logger().info("Logger started.")

    def _emg_callback(self, msg: EMG) -> None:
        self._fifo.write(msg.data.tobytes())

    def __del__(self) -> None:
        os.remove(self._path)
        self.get_logger().info("Logger stopped")


def main():
    rclpy.init()

    logger = Logger()

    try:
        rclpy.spin(logger)
    except BrokenPipeError:
        logger.get_logger().info("FIFO closed.")

    logger.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
