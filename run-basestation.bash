#!/bin/bash

# Initialize VIZ environment variable
VIZ_ENV="false"
RTK_ENV="false"
MOCHA_ENV="false"
SENDER_ENV="false"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --viz)
            VIZ_ENV="true"
            shift # past argument
            ;;
	--rtk)
	    RTK_ENV="true"
	    shift
	    ;;
	--mocha)
	    MOCHA_ENV="true"
	    shift
	    ;;
	--sender)
	    SENDER_ENV="true"
	    shift
	    ;;
	--all)
	    VIZ_ENV="true"
	    RTK_ENV="true"
	    MOCHA_ENV="true"
	    SENDER_ENV="true"
	    shift
	    ;;
        -h|--help)
            echo "Usage: $0 [--viz]"
            echo "  --viz    Runs the geoviz application"
	    echo "  --rtk    Runs the rtk broadcaster"
	    echo "  --mocha  Runs mocha on the basestation"
	    echo "  --sender Runs the scorecard submitter"
	    echo "  --all    Runs everything"
            exit 0
            ;;
        *)
            echo "Unknown option $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

xhost +
docker run -it --rm \
    --network=host \
    --ipc=host \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "./common/config:/home/dtc/ws/src/MOCHA/mocha_core/config" \
    -v "./dtc-msgs:/home/dtc/ws/src/dtc-msgs" \
    -v "./scoring-server-submission/watchstate:/home/dtc/ws/src/MOCHA/interface_rajant/scripts/thirdParty/watchstate" \
    -v "./scoring-server-submission/scorecard_submitter:/home/dtc/ws/src/scorecard_submitter" \
    -v "./basestation-msgs:/home/dtc/ws/src/basestation-msgs" \
    -v "./common/scorecard_parser:/home/dtc/ws/src/scorecard_parser" \
    -v "./rosbags:/home/dtc/ws/rosbags" \  # comment this out if there is no rosbags directory
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e VIZ=$VIZ_ENV \
    -e RTK=$RTK_ENV \
    -e MOCHA=$MOCHA_ENV \
    -e SENDER=$SENDER_ENV \
    --name dtc-ros-jazzy-basestation \
    ros-jazzy:basestation \
    bash
xhost -
