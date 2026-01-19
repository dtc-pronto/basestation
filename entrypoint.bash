#!/bin/bash

HOSTNAME=$(hostname)
source /opt/ros/jazzy/setup.bash
cd ws
colcon build
source install/setup.bash

exec "$@"
# The launch setup uses different namespaces so its not ported for now
# roslaunch basestation basestation.launch mocha:=$MOCHA viz:=$VIZ rtk:=$RTK sender:=$SENDER
