#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Tests vehicle sensors
# - Run this after running the 'start.sh' script

source ~/ros2_ws/install/setup.bash

echo ""
echo "LISTING FOUND TOPICS..."
ros2 topic list

echo ""
echo "TEST COMPLETE"
