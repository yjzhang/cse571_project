#!/bin/bash

ROS_MASTER_URI=http://173.250.246.228:11311

ssh -Y hcrlab@173.250.246.228
roscore &
roslaunch turtlebot_bringup minimal.launch &
rosrun hokuyo_node hokuyo_node
