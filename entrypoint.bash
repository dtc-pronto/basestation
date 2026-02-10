#!/bin/bash

HOSTNAME=$(hostname)
source /opt/ros/jazzy/setup.bash
cd ws
colcon build
source install/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd

exec "$@"
# The launch setup uses different namespaces so its not ported for now
# roslaunch basestation basestation.launch mocha:=$MOCHA viz:=$VIZ rtk:=$RTK sender:=$SENDER
