#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/catkin_ws/devel/setup.bash"

export ROS_IP=`hostname -I|tr -d ' '`

rosrun netatmo2ros main.py _configpath:=/config/
