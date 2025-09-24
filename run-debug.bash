#!/bin/bash

xhost +
docker run -it --rm \
    --network=host \
    --privileged \
    --entrypoint="" \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "./common/config:/home/`whoami`/ws/src/MOCHA/mocha_core/config" \
    -v "./scoring-server-submission/scorecard_submitter:/home/`whoami`/ws/src/scorecard_submitter" \
    -v "./data:/home/`whoami`/ws/data" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e VIZ=$VIZ_ENV \
    -e RTK=$RTK_ENV \
    -e MOCHA=$MOCHA_ENV \
    -e SENDER=$SENDER_ENV \
    --name dtc-basestation \
    dtc-basestation:todos \
    bash
xhost -
