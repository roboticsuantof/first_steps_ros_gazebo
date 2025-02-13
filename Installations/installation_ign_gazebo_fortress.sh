#!/bin/sh

#### General steps for Ignition Gazebo Installation

echo "Calling UPDATE \n\n"
sudo apt update


#### Install Ignition Gazebo Fortress
echo "\n\n INSTALLING RIGNITION GAZEBO FORTRESS \n\n"

echo "\n\n INSTALLING Generic tools \n\n"
sudo apt install python3-pip lsb-release gnupg curl

echo "\n\n INSTALLING vcstool and colcon from pip \n\n"
pip install vcstool || pip3 install vcstool
pip install -U colcon-common-extensions || pip3 install -U colcon-common-extensions
pip show vcstool || pip3 show vcstool | grep Location
pip show colcon-common-extensions || pip3 show colcon-common-extensions | grep Location
export PATH=$PATH:$HOME/.local/bin/

echo "\n\n INSTALLING git \n\n"
sudo apt-get install git

# echo "\n\n INSTALLING vcstool and colcon from git \n\n"
# sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
# curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# sudo apt-get update
# sudo apt-get install python3-vcstool python3-colcon-common-extensions

echo "\n\n Getting the sources \n\n"
mkdir -p ~/ign_gz/src
cd ~/ign_gz/src
curl -O https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-fortress.yaml
vcs import < collection-fortress.yaml

echo "\n\n INSTALLING dependencies \n\n"
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
cd ~/ign_gz/src
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ')

# echo "\n\n INSTALLING compiler requirements \n\n"
# sudo apt-get install g++-8
# sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
# gcc -v
# g++ -v

echo "\n\n BUILDING the Ignition Libraries \n\n"
cd ~/ign_gz/
colcon graph
colcon build --merge-install
# colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
## colcon build --cmake-args ' -DBUILD_TESTING=OFF' ' -DCMAKE_BUILD_TYPE=Debug' --merge-install
## colcon build --merge-install --packages-up-to PACKAGE_NAME
## colcon build --packages-select PACKAGE_NAME


echo "\n\n USING the Workspace \n\n"
. ~/ign_gz/install/setup.bash