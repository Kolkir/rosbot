Setup source list:

1. `echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >ros-latest.list`
2. `sudo cp ros-latest.list /etc/apt/sources.list.d/`

Set up your keys:

1. `sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

Update apt-get:

1. `sudo apt-get update`


Install ROS:

~~~
sudo apt-get install ros-noetic-robot
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
sudo apt-get install ros-noetic-cv-camera
~~~

Setup workspace:

~~~
source /opt/ros/noetic/setup.sh
mkdir ros_workspace
cd ros_workspace/
catkin_make
mkdir src
catkin_make
cd src
git clone https://github.com/Kolkir/rosbot.git
~~~

Setup URIs (required step!!!):
1. `export ROS_MASTER_URI=http://{xxx.xxx.xxx.xxx}:{port}`
2. `export ROS_HOSTNAME=yyy.yyy.yyy.yyy`
