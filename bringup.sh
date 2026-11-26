#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash

export ROS_DOMAIN_ID=42

sudo chmod a+rw /dev/input/js* /dev/input/event*

ros2 launch hexapod_hardware bringup.launch.py total_steps:=80

ros2 launch hexapod_gait gait.launch.py 

ros2 launch hexapod_teleop teleop.launch.py 

ros2 launch hexapod_vision camera.launch.py

ros2 run joy joy_node 
