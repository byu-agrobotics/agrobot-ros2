#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Calls ROS services to start the FSMs
# - Specify a task configuration using 'bash init.sh <task>' (ex. 'bash init.sh sort')

source ~/ros2_ws/install/setup.bash
case $1 in
    "sort")
        ros2 service call "/sort/start" agrobot_interfaces/srv/StartFSM
        ;;
    "navigate")
        ros2 service call /navigate/start agrobot_interfaces/srv/StartFSM
        ;;
    "handle")
        ros2 service call /handle/start agrobot_interfaces/srv/StartFSM
        ;;
    "collect")
        ros2 service call /collect/start agrobot_interfaces/srv/StartFSM
        ;;
    *)
        printError "No task configuration specified"
        printError "Specify an task configuration using 'bash init.sh <task>' (ex. 'bash init.sh sort')"
        ;;
esac
