#!/bin/bash

RTK_PID=""
MOCHA_PID=""

start_rtk() {
    if [ -z "$RTK_PID" ]; then
        echo "Starting RTK broadcaster."

        ros2 launch rtk_correction broadcaster.launch.py \
            ip:=${RTK_IP} \
            port:=${RTK_PORT} &

        RTK_PID=$!
        echo "RTK PID: $RTK_PID"
    fi
}

stop_rtk() {
    if [ -n "$RTK_PID" ]; then
        echo "Stopping RTK broadcaster."
        kill -- -"$RTK_PID" 2>/dev/null || true
        RTK_PID=""
    fi
}

start_mocha() {
    if [ -z "$MOCHA_PID" ]; then
        echo "Starting MOCHA."

        ros2 launch mocha_launch basestation.launch.py \
            robot_name:=basestation &

        MOCHA_PID=$!
        echo "MOCHA PID: $MOCHA_PID"
    fi
}

stop_mocha() {
    if [ -n "$MOCHA_PID" ]; then
        echo "Stopping MOCHA."
        kill -- -"$MOCHA_PID" 2>/dev/null || true
        MOCHA_PID=""
    fi
}

while true; do

    # RTK / F9P check
    if [ "$RTK" == "true" ] || [ -e /dev/ublox ]; then
        start_rtk
    else
        stop_rtk
    fi

    # Rajant check
    if [ "$MOCHA" == "true" ] || ip link show rajant >/dev/null 2>&1; then
        start_mocha
    else
        stop_mocha
    fi

    # Detect crashed processes
    if [ -n "$RTK_PID" ] && ! kill -0 "$RTK_PID" 2>/dev/null; then
        echo "RTK broadcaster exited."
        RTK_PID=""
    fi

    if [ -n "$MOCHA_PID" ] && ! kill -0 "$MOCHA_PID" 2>/dev/null; then
        echo "MOCHA exited."
        MOCHA_PID=""
    fi

    sleep 5
done
