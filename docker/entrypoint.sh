#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

echo "===================================="
echo "ROS2 Humble container"
echo "  container : $(hostname)"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  RMW: $RMW_IMPLEMENTATION"
echo "  /dev/video*: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
echo "  /dev/ttyUSB*: $(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')"
echo "  /root/bags  : $(ls /root/bags 2>/dev/null | tr '\n' ' ')"
echo "===================================="

exec "$@"
