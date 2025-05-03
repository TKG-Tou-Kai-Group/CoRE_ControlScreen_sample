#!/bin/bash

SCRIPTDIR=/home/pi/CoRE_ControlScreen_sample/service/
ENVFILE=/home/pi/CoRE_ControlScreen_sample/ros2_ws/install/local_setup.bash

sudo /usr/sbin/ip link set lo multicast on
echo "Loading ROS2 Env..."
source /opt/ros/humble/setup.bash
source ${ENVFILE}
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "ROS2 Launching..."
exec ros2 launch core_robot_launcher robot_launcher.launch.py 2>&1