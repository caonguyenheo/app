#!/bin/bash

# Load ROS environment
. /opt/ros/kinetic/setup.bash

# execute velodyne
roslaunch ./VLP16_points_multi.launch
