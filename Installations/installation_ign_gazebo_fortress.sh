#!/bin/sh

# General steps for Ignition Gazebo Installation

echo "Calling UPDATE \n\n"
sudo apt update


## Install Ignition Gazebo Fortress
echo "\n\n INSTALLING RIGNITION GAZEBO FORTRESS \n\n"

echo "\n\n STEP 1 \n\n"
sudo apt install python3-pip wget lsb-release gnupg curl
echo "\n\n STEP 2 \n\n"
pip install vcstool || pip3 install vcstool
echo "\n\n STEP 3 \n\n"
pip install -U colcon-common-extensions || pip3 install -U colcon-common-extensions
echo "\n\n STEP 4 \n\n"
pip show vcstool || pip3 show vcstool | grep Location
echo "\n\n STEP 5 \n\n"
pip show colcon-common-extensions || pip3 show colcon-common-extensions | grep Location
echo "\n\n STEP 6 \n\n"
export PATH=$PATH:$HOME/.local/bin/
echo "\n\n STEP 7 \n\n"
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
echo "\n\n STEP 8 \n\n"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "\n\n STEP 9 \n\n"
sudo apt-get update
echo "\n\n STEP 10 \n\n"
sudo apt-get install python3-vcstool python3-colcon-common-extensions
echo "\n\n STEP 11 \n\n"
sudo apt-get install git
echo "\n\n STEP 12 \n\n"
mkdir -p ~/ign_gz_ws/src
echo "\n\n STEP 13 \n\n"
cd ~/ign_gz_ws/src/
echo "\n\n STEP 14 \n\n"
wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-fortress.yaml
echo "\n\n STEP 15 \n\n"
vcs import < collection-fortress.yaml
echo "\n\n STEP 16 \n\n"
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "\n\n STEP 17 \n\n"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
echo "\n\n STEP 18 \n\n"
sudo apt-get update
echo "\n\n STEP 19 \n\n"
cd ~/ign_gz_ws/src/
echo "\n\n STEP 20 \n\n"
sudo apt -y install   $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')
echo "\n\n STEP 21 \n\n"
sudo apt-get install g++-8
echo "\n\n STEP 22 \n\n"
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
echo "\n\n STEP 23 \n\n"
gcc -v
echo "\n\n STEP 1 \n\n"
g++ -v
echo "\n\n STEP 1 \n\n"
cd ~/ign_gz_ws/
echo "\n\n STEP 1 \n\n"
colcon graph
echo "\n\n STEP 1 \n\n"
colcon build --merge-install
echo "\n\n STEP 1 \n\n"
. ~/ign_gz_ws/install/setup.bash
