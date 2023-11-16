#!/bin/bash

#Upgrade python (ROS melodic uses 2.7 we have to use at least 3.8 for our service)
sudo apt install python3.8

#Install dependencies
python3.8 -m pip upgrade pip opencv-python mediapipe PyYaml catkin_pkg ros_pkg


