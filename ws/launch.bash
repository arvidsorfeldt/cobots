#!/usr/bin/env bash
source /opt/ros/galactic/setup.bash
source ../dep_ws/install/local_setup.bash
colcon build
source install/local_setup.bash

ros2 launch case_bringup case_r4.launch.py