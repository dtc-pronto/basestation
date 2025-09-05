#!/usr/bin/env bash
set -euo pipefail

# DISPLAY can be overridden by `-e DISPLAY=$DISPLAY` at runtime
echo "[entrypoint] DISPLAY=${DISPLAY:-unset}"
exec python3 /app/viewer_gui.py
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}
