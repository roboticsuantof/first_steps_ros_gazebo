#!/bin/sh

# General stup NUC for marsupial agents

echo "Calling UPDATE \n\n"

sudo apt update

## Install ROS Noetic
echo "\n\n INSTALLING ROS_NOETIC \n\n"

echo "\n\n STEP 1 \n\n"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "\n\n STEP 2 \n\n"
sudo apt install curl # if you haven't already installed curl
echo "\n\n STEP 3 \n\n"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "\n\n STEP 4 \n\n"
sudo apt update 
echo "\n\n STEP 5 \n\n"
sudo apt install ros-noetic-desktop-full
echo "\n\n STEP 6 \n\n"
source /opt/ros/noetic/setup.bash
echo "\n\n STEP 7 \n\n"
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "\n\n STEP 8 \n\n"
source ~/.bashrc
echo "\n\n STEP 9 \n\n"
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
echo "\n\n STEP 10 \n\n"
sudo apt install python3-rosdep
echo "\n\n STEP 11 \n\n"
sudo rosdep init
echo "\n\n STEP 12 \n\n"
rosdep update