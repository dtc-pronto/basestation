#!/bin/bash

docker build --no-cache --build-arg user_id=$(id -u) --rm -t dtc-platform-`hostname`:basestation .
