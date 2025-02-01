#!/bin/bash
# Created by Ashton Palacios, Dec 2024
#
# Quick egg identification test

source ~/ros2_ws/install/setup.bash

echo ""
ros2 service call /egg/identify agrobot_interfaces/srv/IdentifyEgg
echo ""
