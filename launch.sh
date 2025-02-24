#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Starts the micro-ROS agent and ROS 2 launch files
# - Specify a task configuration using 'bash launch.sh <task>' (ex. 'bash launch.sh sort')

export PARAM_FILE=/home/agrobot/config/robot_params.yaml

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

# Launch the specified task
case $1 in
    "sort")
        printInfo "Starting the sort task nodes..."
        ros2 launch task_fsm sort_launch.py param_file:=$PARAM_FILE
        ;;
    "navigate")
        printInfo "Starting the navigate task nodes..."
        ros2 launch task_fsm navigate_launch.py param_file:=$PARAM_FILE
        ;;
    "handle")
        printInfo "Starting the handle task nodes..."
        ros2 launch task_fsm handle_launch.py param_file:=$PARAM_FILE
        ;;
    "collect")
        printInfo "Starting the collect task nodes..."
        ros2 launch task_fsm collect_launch.py param_file:=$PARAM_FILE
        ;;
    *)
        printError "No task configuration specified"
        printError "Specify a task configuration using 'bash launch.sh <task>' (ex. 'bash launch.sh sort')"
        ;;
esac
