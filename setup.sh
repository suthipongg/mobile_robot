#!/bin/sh

# setup catkin_ws path
echo "================================ Setup catkin_ws path ================================"
CATKIN_WS=$HOME/catkin_ws
CATKIN_WS_src=$CATKIN_WS/src
mkdir -p $CATKIN_WS_src
dir_setup=$(dirname "$0")
curr_dir=$(pwd)
mv $dir_setup/* $dir_setup/.g* $CATKIN_WS_src
rm -rf $dir_setup

# install ROS noetic
# ref : http://wiki.ros.org/noetic/Installation/Ubuntu
echo "================================ install ROS noetic ================================"
echo ">>>>>>>>>>>>>> Setup your sources.list"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

echo ">>>>>>>>>>>>>> Set up your keys"
apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

echo ">>>>>>>>>>>>>> Installation"
apt update
apt install ros-noetic-desktop-full -y
source /opt/ros/noetic/setup.bash

echo ">>>>>>>>>>>>>> install Dependencies for building packages"
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
echo "--------- Initialize rosdep"
rosdep init
rosdep update

echo "================================ Build navigation ================================"
echo ">>>>>>>>>>>>>> install Dependencies for navigation system"
echo "--------- Install SDL_LIBRARY SDL_INCLUDE_DIR"
apt-get install libsdl-image1.2-dev -y
apt-get install libsdl-dev -y
echo "--------- Install SuiteSparse"
apt-get install -y libsuitesparse-dev -y
echo "--------- Install libg20"
apt-get install ros-noetic-libg2o -y
echo "--------- Install WiringPi"
apt install libxmlrpcpp-dev -y
apt install librosconsole-dev -y
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi && ./build
cd .. && rm -rf WiringPi

echo ">>>>>>>>>>>>>> catkin make"
cd $CATKIN_WS
catkin_make

echo "================================ setup .bashrc ================================"
echo "
CATKIN_WS=\$HOME/catkin_ws
CATKIN_WS_BASH=\$CATKIN_WS/devel/setup.bash
alias eb='vim \$HOME/.bashrc'
alias sb='source \$HOME/.bashrc'
alias gs='git status'
alias cw='cd \$CATKIN_WS'
alias cs='cd \$CATKIN_WS/src'
alias cm='cw && catkin_make'

source /opt/ros/noetic/setup.bash
source \$CATKIN_WS_BASH

IP_SERVER=\"\$(hostname -I | cut -d ' ' -f 1)\"
if [ \"\$IP_SERVER\" = \"\" ]; then
  IP_SERVER='127.0.0.1'
fi

export ROS_IP=\$IP_SERVER
export ROS_MASTER_URI=http://\$IP_SERVER:11311" >> ~/.bashrc
source ~/.bashrc

apt-get update
apt-get upgrade -y
apt autoremove
sb
