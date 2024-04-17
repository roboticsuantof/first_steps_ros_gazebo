#!/bin/sh

# General steps for Ignition Gazebo Installation

echo "Calling UPDATE \n\n"
sudo apt update


## Install Ignition Gazebo Fortress
echo "\n\n INSTALLING RIGNITION GAZEBO FORTRESS \n\n"

sudo apt install python3-pip wget lsb-release gnupg curl
pip install vcstool || pip3 install vcstool
pip install -U colcon-common-extensions || pip3 install -U colcon-common-extensions
pip show vcstool || pip3 show vcstool | grep Location
pip show colcon-common-extensions || pip3 show colcon-common-extensions | grep Location
export PATH=$PATH:$HOME/.local/bin/
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
sudo apt-get install git
mkdir -p ~/ign_gz_ws/src
cd ign_gz_ws/src/
wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-fortress.yaml
vcs import < collection-fortress.yaml
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
cd ~/ign_gz_ws/src/
sudo apt -y install   $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')
sudo apt-get install g++-8
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
gcc -v
g++ -v
cd ~/ign_gz_ws/
colcon graph
colcon build --merge-install
. ~/ign_gz_ws/install/setup.bash
