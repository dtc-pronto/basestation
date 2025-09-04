#!/bin/bash

docker build \
    --no-cache \
    --build-arg user_id=$(id -u) \
    --build-arg USER=$(whoami) \
    --build-arg NAME=$(hostname) \
    --rm \
    -t dtc-basestation:scorecard .
