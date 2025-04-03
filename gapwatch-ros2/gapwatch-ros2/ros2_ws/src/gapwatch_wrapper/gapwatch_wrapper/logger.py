"""
Object that logs EMG data to a TCP socket.


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
import time

import rclpy
from gapwatch_messages.msg import EMG
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class Logger(Node):

    def __init__(self) -> None:

        super().__init__("logger")
        self._emg_subscription = self.create_subscription(
            EMG, "emg", self._emg_callback, 100
        )
        self._emg_subscription

        # Open socket
        self.declare_parameter(
            "server_addr",
            "172.17.0.1",
            ParameterDescriptor(description="Server address"),
        )
        self.declare_parameter(
            "server_port",
            3334,
            ParameterDescriptor(description="Server port"),
        )
        addr = self.get_parameter("server_addr").get_parameter_value().string_value
        port = self.get_parameter("server_port").get_parameter_value().integer_value

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while True:
            try:
                self._sock.connect((addr, port))
                break
            except ConnectionRefusedError:
                self.get_logger().info("Connection refused, retrying in a second...")
                time.sleep(1)
        self.get_logger().info(f"Connected to server at {addr}:{port}.")

    def _emg_callback(self, msg: EMG) -> None:
        self._sock.sendall(msg.emg.tobytes())
        self._sock.sendall(msg.battery.tobytes())
        self._sock.sendall(msg.counter.tobytes())
        self._sock.sendall(msg.ts.tobytes())

    def __del__(self) -> None:
        self._sock.close()


def main():
    rclpy.init()
    logger = Logger()

    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("Manual shutdown.")
    except BrokenPipeError:
        print("Connection closed by server.")
    finally:
        # Shutdown
        if rclpy.ok():
            logger.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
