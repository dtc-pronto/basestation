#!/bin/bash

HOSTNAME=$(hostname)
source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash

roslaunch basestation basestation.launch mocha:=$MOCHA viz:=$VIZ rtk:=$RTK sender:=$SENDER
