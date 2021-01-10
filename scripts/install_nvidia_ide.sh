#!/usr/bin/env sh
apt-get update
apt-get install -y lsb-release
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt-get update
apt-get install -y ros-noetic-desktop-full
apt-get install -y qtcreator
apt-get install -y clang-format
apt-get install -y libclang-common-8-dev
apt-get install -y vim
apt-get install -y tmux
apt-get install -y mc
apt-get install -y git
apt-get install -y git-gui
apt-get install -y ros-noetic-video-stream-opencv
apt-get install -y ros-noetic-teleop-twist-keyboard
apt-get install -y ros-noetic-rosparam-shortcuts
apt-get install -y ros-noetic-rqt-robot-steering
apt-get install -y ros-noetic-gazebo-ros-pkgs
apt-get install -y ros-noetic-gazebo-ros-control
apt-get install -y ros-noetic-ros-controllers
apt-get install -y ros-noetic-genpy
apt-get install -y libgpiod-dev
apt-get install -y ros-noetic-costmap-2
apt-get install -y ros-noetic-octomap-msgs
apt-get install -y ros-noetic-octomap-server
