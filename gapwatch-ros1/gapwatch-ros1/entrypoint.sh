#!/bin/bash

# Source ROS1 and compile nodes
source /opt/ros/noetic/setup.bash
catkin_make

# Run ROS1 node
source devel/setup.bash
roscore &
rosrun "$@"
