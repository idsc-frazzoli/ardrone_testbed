#!/bin/bash
source ~/ardrone_ws/devel/setup.bash
cd ~/ardrone_ws/src/ardrone_glc/ORB_SLAM2/
./build_ros.sh
cd ~/ardrone_ws/
catkin_make
