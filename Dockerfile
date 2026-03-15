FROM dtcpronto/ros-jazzy:full

RUN sudo apt-get update && sudo apt-get install -y \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-vision-opencv \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    python3-utm \
    python3-flask \
    python3-folium \
    python3-flask-socketio \
    python3-scipy \
    python3-dotenv \
    python3-requests 

RUN cd ws/src && git clone https://github.com/KumarRobotics/MOCHA -b dtc/ros2/main
RUN cd ws/src && git clone https://github.com/dtc-pronto/dtc-msgs
RUN cd ws/src && git clone https://github.com/dtc-pronto/rtk-correction
WORKDIR /home/dtc

ENV MOCHA=false \
    RTK=false \
    SENDER=false \
    GATE=1

RUN sudo apt update && sudo apt install -y \
 python3-zmq \
 default-jre \
 iputils-ping \
 python3-lz4 \
 python3-defusedxml

COPY ./scoring-server-submission ws/src/scoring-server-submission
RUN sudo apt install ros-jazzy-smach
COPY ./watchstate ws/src/MOCHA/interface_rajant/thirdParty/watchstate

RUN /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    cd /home/dtc/ws && \
    colcon build"

RUN echo ". /opt/ros/jazzy/setup.bash" >> /home/dtc/.bashrc \
 && echo "source /home/dtc/ws/install/setup.bash" >> /home/dtc/.bashrc \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> /home/dtc/.bashrc

COPY --chown=dtc:dtc ./entrypoint.bash /home/dtc/entrypoint.bash
RUN chmod +x /home/dtc/entrypoint.bash

ENTRYPOINT ["/home/dtc/entrypoint.bash"]
