#!/bin/bash

# Initialize VIZ environment variable
RTK_ENV="false"
MOCHA_ENV="false"
SENDER_ENV="false"
GATE_ENV="1"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
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
            RTK_ENV="true"
            MOCHA_ENV="true"
            SENDER_ENV="true"
            shift
            ;;
        -h|--help)
                echo "Usage: $0 [--viz]"
                echo "  --rtk    Runs the rtk broadcaster"
                echo "  --mocha  Runs mocha on the basestation"
                echo "  --sender Runs the scorecard submitter"
                echo "  --all    Runs everything"
                echo "  --gate N Sets the gate number (default: 1)"
                exit 0
                ;;
       --gate)
            if [[ -n "$2" && "$2" =~ ^[0-9]+$ ]]; then
                GATE_ENV="$2"
                shift 2
            else
                echo "Error: --gate requires a numeric argument"
                exit 1
            fi
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
    --entrypoint="" \
    -v "/dev:/dev" \
    -v "./data:/home/dtc/data" \
    -v "./scoring-server-submission:/home/dtc/ws/src/scoring-server-submission" \
    -e RTK=$RTK_ENV \
    -e MOCHA=$MOCHA_ENV \
    -e SENDER=$SENDER_ENV \
    -e GATE=$GATE_ENV \
    --name dtc-platform-`hostname`-basestation \
    dtc-platform-`hostname`:basestation \
    bash
xhost -
