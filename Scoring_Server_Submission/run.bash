#!/bin/bash

xhost +
docker run -it --rm \
    --network=host \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "./scorecard_submitter:/home/`whoami`/ws/src/scorecard_submitter" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name dtc-basestation \
    dtc-basestation:scorecard \
    bash
xhost -
