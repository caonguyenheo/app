#!/bin/bash

# Load ROS environment from setup.sh
COLCON_CURRENT_PREFIX=/home/dong/ros2-crytal-x64

#Load ROS environment
. /opt/ros/kinetic/setup.bash

# Load ROS2 environment
. /home/dong/ros2-crytal-x64/setup.sh

# Load katri_msgs library
. /home/dong/qt-projects/katri-logging-system/katri_msgs/install/setup.bash

# execute katri logger
/home/dong/qt-projects/katri-logging-system/build-katri-logger-Desktop_Qt_5_13_0_GCC_64bit-Debug/katri_logger
