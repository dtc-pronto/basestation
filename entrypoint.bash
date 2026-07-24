#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ws/install/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd &
set -e
echo "RTK_IP:" ${RTK_IP}

GATE1="false"
GATE2="false"
GATE3="false"
GATE4="false"
GATE5="false"
if [ "$GATE" == "1" ]; then
    echo "Running Gate 1"
    GATE1="true"
elif [ "$GATE" == "2" ]; then
    echo "Running Gate 2"
    GATE2="true"
elif [ "$GATE" == "3" ]; then
    echo "Running Gate 3"
    GATE3="true"
elif [ "$GATE" == "4" ]; then
    echo "Running Gate 4"
    GATE4="true"
elif [ "$GATE" == "5" ]; then
    echo "Running HMT Gate"
    GATE5="true"
fi

if [ "$SENDER" == "true" ]; then
    echo "Launching Scorecard Sender"
    ros2 launch scorecard_submitter gate_conditional.launch.py \
        gate_1:=${GATE1} \
        gate_2:=${GATE2} \
        gate_3:=${GATE3} \
        gate_4:=${GATE4} \
        hmt:=${GATE5} &
fi

if [ "$MOCHA" == "true" ]; then
    echo "Launching MOCHA"
    ros2 launch mocha_launch basestation.launch.py robot_name:=basestation &
fi

if [ "$RTK" == "true" ]; then
    echo "Launching RTK Broadcaster"
    ros2 launch rtk_correction broadcaster.launch.py ip:=${RTK_IP} port:=${RTK_PORT} &
fi

wait
