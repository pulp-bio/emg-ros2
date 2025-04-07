#!/usr/bin/env python3

from __future__ import annotations

import socket
import struct
import time
from functools import partial

import rospy
from gapwatch_streamer.msg import EMG


def callback(msg: EMG, sock: socket) -> None:
    try:
        sock.sendall(struct.pack(f"{len(msg.emg)}f", *msg.emg))
        sock.sendall(struct.pack("B", msg.battery))
        sock.sendall(struct.pack("B", msg.counter))
        sock.sendall(struct.pack("Q", msg.ts))
    except BrokenPipeError:
        rospy.logerr("Shutdown by server.")
        rospy.signal_shutdown("Shutdown by server.")


def close(sock: socket) -> None:
    rospy.loginfo("Shutting down node...")
    sock.close()


def main():
    # Initialize the ROS node
    rospy.init_node("logger", anonymous=True)

    # Open socket for BioGUI logging
    server_addr = rospy.get_param("~server_addr", "172.17.0.1")
    server_port = rospy.get_param("~server_port", 3334)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            sock.connect((server_addr, server_port))
            break
        except ConnectionRefusedError:
            rospy.loginfo("Connection refused, retrying in a second...")
            time.sleep(1)
    rospy.loginfo(f"Connected to server at {server_addr}:{server_port}.")

    # Subscribe to EMG topic
    rospy.Subscriber("/emg", EMG, partial(callback, sock=sock))

    # Close socket on exit
    rospy.on_shutdown(partial(close, sock=sock))

    # Spin to keep the script for exiting
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
