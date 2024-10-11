#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts a ROS2 bag recording and initializes the vehicle
# - Run this after running the 'start.sh' and 'test.sh'
#   scripts
# - Log files are saved in '../bag' on the host machine
#   running the docker container

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

echo ""
echo "Enter a descriptive folder name for the rosbag: "
read FOLDER
echo ""

source ~/ros2_ws/install/setup.bash

ros2 topic pub /init std_msgs/msg/Empty -1

FOLDER=$FOLDER-$(date +"%Y-%m-%d-%H-%M-%S")
ros2 bag record -o ~/bag/$FOLDER -s mcap -a
