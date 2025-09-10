#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-udp-viewer:latest}"
BASE_NAME="${BASE_NAME:-udp-viewer}"
COUNT="${1:-2}"                # how many instances
BASE_PORT="${BASE_PORT:-27000}" # first direct receive port
BASE_BRIDGE="${BASE_BRIDGE:-27100}" # first bridge port

# Allow X for Docker once
xhost +local:docker >/dev/null 2>&1 || true

for i in $(seq 0 $((COUNT-1))); do
  name="${BASE_NAME}-$((i+1))"
  recv_port=$((BASE_PORT + i))
  bridge_port=$((BASE_BRIDGE + i))

  echo "Launching $name (recv:$recv_port, bridge:$bridge_port)"
  docker run --rm -d \
    --network=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e LIBGL_DRI3_DISABLE=1 \
    -e VDPAU_DRIVER=dummy \
    --name "$name" \
    "$IMAGE_NAME"
done

# Revoke after you're done manually, or let your existing run.bash revoke when each exits:
# xhost -local:docker
