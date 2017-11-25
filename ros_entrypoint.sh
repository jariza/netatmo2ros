#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/catkin_ws/devel/setup.bash"

export ROS_IP=`hostname -I`

rosrun netatmo2ros main.py _configfile:=/config/config.ini
