#!/bin/sh

#### General steps for ROS Ignition Gazebo Briedge Installation

echo "Calling UPDATE \n\n"
sudo apt update

export IGNITION_VERSION=fortress

# Setup the workspace
echo "Calling Setup workspace \n\n"
mkdir -p ~/ros_ign_gz_ws/src
cd ~/ros_ign_gz_ws/src

# Download needed software
echo "Downloading Repository \n\n"
git clone https://github.com/osrf/ros_ign.git -b noetic

#installation
echo "Installation Repository \n\n"
cd ~/ros_ign_gz_ws
rosdep install -r --from-paths src -i -y --rosdistro noetic

# Source ROS distro's setup.bash
source /opt/ros/noetic/setup.bash

# Build and install into workspace
echo "Boulding Repository \n\n"
cd ~/ros_ign_gz_ws/
catkin_make install