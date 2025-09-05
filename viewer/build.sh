#!/usr/bin/env bash
set -e
IMAGE=${IMAGE:-udp-viewer:latest}
docker build -t "$IMAGE" .
echo "Built $IMAGE"
