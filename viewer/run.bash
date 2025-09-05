#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-udp-viewer:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-udp-viewer-gui}"

# Check DISPLAY
if [[ -z "${DISPLAY:-}" ]]; then
  echo "[run.bash] DISPLAY is empty. Try: export DISPLAY=:0"
  exit 1
fi

# Allow Docker to talk to your X server
xhost +local:docker >/dev/null 2>&1 || true

# Run the viewer
docker run --rm -it \
  --network=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  --name "${CONTAINER_NAME}" \
  "${IMAGE_NAME}"

# Revoke access after exit
xhost -local:docker >/dev/null 2>&1 || true

