#!/usr/bin/env sh
xhost +local:root
docker run --device /dev/video1:/dev/video0 --device /dev/dri --ipc=host --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -dt --name robot_env --restart unless-stopped -v `pwd`:/ros_workspace osrf/ros:noetic-desktop-full   
