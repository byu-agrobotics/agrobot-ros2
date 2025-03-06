#!/bin/bash
# Created by Ashton Palacios, Dec 2024
#
# Quick egg identification test

echo ""
ros2 service call /egg/identify agrobot_interfaces/srv/IdentifyEgg
echo ""
