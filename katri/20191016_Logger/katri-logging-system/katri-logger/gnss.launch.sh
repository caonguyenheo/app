#!/bin/bash

# Load ROS environment
. /opt/ros/kinetic/setup.bash

# execute velodyne
roslaunch ./GNSS_usb_serial.launch
