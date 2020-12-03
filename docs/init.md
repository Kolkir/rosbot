### Configure environment

Get Docker image:

`docker pull osrf/ros:noetic-desktop-full`

Start detached container, with mounted workspace directory, spcified name, shared network, web cam, and X11 :
~~~
cd my_workspace_directory`
docker run --device=/dev/video0:/dev/video0 --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -dt --name robot_env --restart unless-stopped -v `pwd`:/root/workspace osrf/ros:noetic-desktop-full
~~~
Connect to the contatiner:

`docker exec -it robot_env bash`

Stop and delete contatiner:

`docker stop robot_env && docker rm robot_env`

you can also stop and then start container again.

### Start development

Initialize workspace:
~~~
cd root/workspace
source /opt/ros/noetic/setup.bash
mkdir src
catkin_make
source devel/setup.bash
~~~
Check workspace:

`echo $ROS_PACKAGE_PATH`

### Configure `~/.bashrc`
~~~
# Set ROS Kinetic
source /opt/ros/noetic/setup.bash
source /root/workspace/devel/setup.bash

# Set ROS Network
export ROS_HOSTNAME=xxx.xxx.xxx.xxx
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311

# Set ROS alias command
alias cw='cd /root/workspace'
alias cs='cd /root/workspace/src'
alias cm='cd /root/workspace && catkin_make'
~~~

### Install IDE
~~~
apt-get update
apt-get install qtcreator
apt-get install vim
apt-get install tmux
~~~

### Install ROS packages

`apt-get install ros-noetic-cv-camera`

or use `rosdep install package_name`

### Camera

http://wiki.ros.org/stereo_image_proc/Tutorials/Using%20OpenCV%20for%20Image%20Processing#cv_camera

~~~
cd /root/workspace/src
catkin_create_pkg main_camera cv_camera

roslaunch main_camera main_camera.launch main_camera_id:=1
~~~