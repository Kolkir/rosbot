Setup source list:

1. `echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >ros-latest.list`
2. `sudo cp ros-latest.list /etc/apt/sources.list.d/`

Set up your keys:

1. `sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

Update apt-get:

1. `sudo apt-get update`


Install ROS:

1. sudo apt-get install ros-noetic-robot

Configure ROS master URL:

1. `export ROS_MASTER_URI=http://xxx.xxx.xxx.xxx:11311`

