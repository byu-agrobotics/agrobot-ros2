#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Calls ROS services to start the FSMs
# - Specify a task configuration using 'bash init.sh <task>' (ex. 'bash init.sh sort')

source ~/ros2_ws/install/setup.bash

case $1 in
    "sort")
        ros2 service call /fsm/sort std_srvs/srv/Trigger # TODO: Add a sort service
        ;;
    "navigate")
        ros2 service call /fsm/navigate std_srvs/srv/Trigger # TODO: Add a navigate service
        ;;
    "handle")
        ros2 service call /fsm/handle std_srvs/srv/Trigger # TODO: Add a handle service
        ;;
    "collect")
        ros2 service call /fsm/collect std_srvs/srv/Trigger # TODO: Add a collect service
        ;;
    *)
        printError "No task configuration specified"
        printError "Specify an task configuration using 'bash init.sh <task>' (ex. 'bash init.sh sort')"
        ;;
esac