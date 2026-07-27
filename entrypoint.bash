#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash
source /home/dtc/ws/install/setup.bash

echo "Starting Zenoh router"
ros2 run rmw_zenoh_cpp rmw_zenohd &

echo "RTK_IP: ${RTK_IP}"
echo "RTK_PORT: ${RTK_PORT}"

GATE1="false"
GATE2="false"
GATE3="false"
GATE4="false"
GATE5="false"

case "$GATE" in
    1)
        echo "Running Gate 1"
        GATE1="true"
        ;;
    2)
        echo "Running Gate 2"
        GATE2="true"
        ;;
    3)
        echo "Running Gate 3"
        GATE3="true"
        ;;
    4)
        echo "Running Gate 4"
        GATE4="true"
        ;;
    5)
        echo "Running HMT Gate"
        GATE5="true"
        ;;
esac


# Manual scorecard sender
if [ "$SENDER" == "true" ]; then
    echo "Launching Scorecard Sender"

    ros2 launch scorecard_submitter gate_conditional.launch.py \
        gate_1:=${GATE1} \
        gate_2:=${GATE2} \
        gate_3:=${GATE3} \
        gate_4:=${GATE4} \
        hmt:=${GATE5} &
fi


# Manual MOCHA
if [ "$MOCHA" == "true" ]; then
    echo "Launching MOCHA"

    ros2 launch mocha_launch basestation.launch.py \
        robot_name:=basestation &
fi


# Manual RTK
if [ "$RTK" == "true" ]; then
    echo "Launching RTK Broadcaster"

    ros2 launch rtk_correction broadcaster.launch.py \
        ip:=${RTK_IP} \
        port:=${RTK_PORT} &
fi


# Hardware autonomous mode
if [ "$RTK" == "false" ] && [ "$MOCHA" == "false" ] && [ "$SENDER" == "false" ]; then
    echo "No manual services selected"
    echo "Starting basestation hardware supervisor"

    /home/dtc/basestation-supervisor.sh &
fi


echo "Basestation running"
wait
