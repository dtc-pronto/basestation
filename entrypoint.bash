#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ws/install/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd&

GATE1="false"
GATE2="false"
GATE3="false"
GATE4="false"
GATE5="false"

if [ "$GATE_ENV" == "1" ]; then
    echo "Running Gate 1"
    GATE1="true"
elif [ "$GATE_ENV" == "2" ]; then
    echo "Running Gate 2"
    GATE2="true"
elif [ "$GATE_ENV" == "3" ]; then
    echo "Running Gate 3"
    GATE3="true"
elif [ "$GATE_ENV" == "4" ]; then
    echo "Running Gate 4"
    GATE4="true"
elif [ "$GATE_ENV" == "5" ]; then
    echo "Running HMT Gate"
    GATE5="true"
fi

if [ "$SENDER" == "true" ]; then
    echo "Launching Scorecard Sender"
    ros2 launch scorecard_submitter gate_conditional.launch.py gate_1:=${GATE1} gate_2:=${GATE2} gate_3:=${GATE3} gate_4:=${GATE4} hmt:=${GATE5} &
fi

if [ "$MOCHA" == "true" ]; then
    echo "Launching MOCHA"
    ros2 launch mocha_launch basestation.launch.py robot_name:=basestation &
fi

if [ "$RTK" == "true" ]; then
    echo "Launching RTK Broadcaster"
    # this will launch with default ip address of 10.10.10.10 and port 7507
    ros2 launch rtk_correction broadcaster.launch.py &
fi
wait
