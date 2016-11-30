#!/bin/bash

source /opt/ros/indigo/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export LIBGL_ALWAYS_SOFTWARE=1

roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/indigo/share/turtlebot_gazebo/worlds/corridor.world
#python goforward.py # run this after the turtlebot has started
