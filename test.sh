#!/bin/bash
# Created by Nelson Durrant, Sep 2024
#
# Tests vehicle sensors and actuators
# - Run this after running the 'launch.sh' script

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

source ~/ros2_ws/install/setup.bash

echo ""
ros2 topic list # TODO: Make this more specific
echo ""
