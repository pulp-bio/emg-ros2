#!/bin/bash


# Workaround for setuptools
PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"
export PYTHONWARNINGS

# Source ROS2 and compile nodes
source /opt/ros/humble/setup.bash
colcon build

# Run ROS2 node
source install/setup.bash
ros2 run gapwatch_wrapper "$@"
