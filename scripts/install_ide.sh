#!/usr/bin/env sh
apt-get update
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

ln -s /usr/bin/python3 /usr/bin/python
