#!/bin/bash

docker build --build-arg user_id=$(id -u) --rm -t ros-jazzy:basestation -f Dockerfile.jazzy.basestation .
