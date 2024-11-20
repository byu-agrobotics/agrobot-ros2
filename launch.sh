#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts the micro-ROS agent and ROS 2 launch files
# - Specify a task configuration using 'bash launch.sh <task>' (ex. 'bash launch.sh sort')

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
echo "################################################################"
echo "# BYU AGRICULTURAL ROBOTICS TEAM - STARTING THE AGROBOT SYSTEM #"
echo "################################################################"
echo ""

# Test for Teensy board connection
if [ -z "$(tycmd list | grep Teensy)" ]; then
    printError "No Teensy boards avaliable to connect to"
    exit 1
fi

# Start both workspaces
source ~/microros_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
case $1 in
    "sort")
        ros2 launch agrobot sort_launch.py
        ;;
    "navigate")
        ros2 launch agrobot navigate_launch.py
        ;;
    "handle")
        ros2 launch agrobot handle_launch.py
        ;;
    "collect")
        ros2 launch agrobot collect_launch.py
        ;;
    *)
        printError "No task configuration specified"
        printError "Specify a task configuration using 'bash launch.sh <task>' (ex. 'bash launch.sh sort')"
        ;;
esac
