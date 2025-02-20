#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Calls ROS services to start the FSMs
# - Specify a task configuration using 'bash init.sh <task>' (ex. 'bash init.sh sort')

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

# Trigger the specified task
case $1 in
    "sort")
        printInfo "Triggering the sort task..."
        ros2 service call /sort/start agrobot_interfaces/srv/StartFSM
        ;;
    "navigate")
        printInfo "Triggering the navigate task..."
        ros2 service call /navigate/start agrobot_interfaces/srv/StartFSM
        ;;
    "handle")
        printInfo "Triggering the handle task..."
        ros2 service call /handle/start agrobot_interfaces/srv/StartFSM
        ;;
    "collect")
        printInfo "Triggering the collect task..."
        ros2 service call /collect/start agrobot_interfaces/srv/StartFSM
        ;;
    *)
        printError "No task configuration specified"
        printError "Specify an task configuration using 'bash init.sh <task>' (ex. 'bash init.sh sort')"
        ;;
esac
