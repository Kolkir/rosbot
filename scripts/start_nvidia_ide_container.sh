#!/usr/bin/env sh
xhost +local:root
docker run --gpus all --device /dev/dri --ipc=host --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -dt --name robot_env --restart unless-stopped -v `pwd`:/ros_workspace nvidia/cudagl:11.1-devel   
