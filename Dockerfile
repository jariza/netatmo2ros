# Base container: ros
FROM ros

# Configuration volume
VOLUME /config

# Required Python library
RUN apt-get update && apt-get install -y python-requests

# Create workspace and copy required files
RUN mkdir -p /home/catkin_ws/src/netatmo2ros
WORKDIR /home/catkin_ws
COPY . /home/catkin_ws/src/netatmo2ros/

# Compile ROS node
RUN /bin/bash -c " source /opt/ros/$ROS_DISTRO/setup.bash; /opt/ros/kinetic/bin/catkin_make"

# Copy entrypoint file
COPY ros_entrypoint.sh /ros_entrypoint.sh
