source ~/ros2_ws/install/setup.bash

echo ""
ros2 service call /egg/identify agrobot_interfaces/srv/IdentifyEgg
echo ""
