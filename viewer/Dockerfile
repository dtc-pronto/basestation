# Small, modern base
FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# System deps for ffplay + X11 rendering
# System deps for ffplay + X11 rendering
RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg \
    # X11 + xcb stack needed by Qt
    libx11-6 libx11-xcb1 libxext6 libxrender1 libxfixes3 libxi6 \
    libxcb1 libxcb-image0 libxcb-icccm4 libxcb-keysyms1 \
    libxcb-render0 libxcb-shape0 libxcb-shm0 libxcb-xfixes0 libxcb-randr0 \
    libxkbcommon-x11-0 libxcb-cursor0 \
    # GL + misc that Qt may touch
    libgl1 libegl1 libdbus-1-3 xauth \
    # fonts (some Qt builds choke without fontconfig/fonts)
    libgl1-mesa-dri \
    libsdl2-2.0-0 \   
    libfontconfig1 fonts-dejavu-core \
  && rm -rf /var/lib/apt/lists/*

# Python deps (PyQt6 ships Qt in wheels)
RUN pip install --no-cache-dir PyQt6

# App
WORKDIR /app
COPY viewer_gui.py /app/viewer_gui.py
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh /app/viewer_gui.py

# Helpful defaults for X11
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

ENTRYPOINT ["/app/entrypoint.sh"]
