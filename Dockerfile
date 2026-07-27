FROM dtcpronto/ros-jazzy:full

WORKDIR /home/dtc

ENV MOCHA=false \
    RTK=false \
    SENDER=false \
    GATE=1

RUN sudo apt-get update && sudo apt-get install -y \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-vision-opencv \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-smach \
    python3-utm \
    python3-flask \
    python3-folium \
    python3-flask-socketio \
    python3-scipy \
    python3-dotenv \
    python3-requests \
    python3-zmq \
    python3-serial \
    python3-lz4 \
    python3-defusedxml \
    iputils-ping \
    iproute2 \
    default-jre \
 && sudo rm -rf /var/lib/apt/lists/*

RUN sudo pip3 install --no-cache-dir \
    --break-system-packages \
    pyrtcm

# Common packages
COPY common/dtc-msgs              ws/src/dtc-msgs
COPY common/rtk-correction        ws/src/rtk-correction
COPY common/MOCHA                 ws/src/MOCHA

# Local packages
COPY scoring-server-submission    ws/src/scoring-server-submission
COPY watchstate                   ws/src/MOCHA/interface_rajant/thirdParty/watchstate

RUN /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    cd /home/dtc/ws && \
    colcon build"

RUN echo ". /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /home/dtc/ws/install/setup.bash" >> ~/.bashrc && \
    echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc

COPY --chown=dtc:dtc entrypoint.bash /home/dtc/
COPY --chown=dtc:dtc scripts/basestation-supervisor.sh /home/dtc/

RUN chmod +x \
    /home/dtc/entrypoint.bash \
    /home/dtc/basestation-supervisor.sh

USER root
ENTRYPOINT ["/home/dtc/entrypoint.bash"]
