#!/bin/bash

# Load ROS environment
. /opt/ros/kinetic/setup.bash

# Load ouster driver
source /home/dong/git/ouster_ws/devel/setup.bash

# execute ouster
roslaunch ouster_driver os1.launch lidar_address:=192.168.1.145 pc_address:=192.168.1.254
