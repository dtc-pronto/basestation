FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# install the basics
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
 vim \
 tmux \
 cmake \
 gcc \
 g++ \
 git \
 build-essential \
 sudo \
 wget \
 curl \
 zip \
 unzip

# add a user
ARG user_id
ARG USER fossa
RUN useradd -U --uid ${user_id} -ms /bin/bash $USER \
 && echo "$USER:$USER" | chpasswd \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# add groups to get access to sensors
USER $USER
WORKDIR /home/$USER
RUN mkdir -p ws/src

RUN sudo apt-get install -y python3-catkin-tools

# install visualizer deps
RUN sudo apt-get install -y python3-pip
RUN pip3 install flask folium flask_socketio utm
RUN pip3 install scipy python-dotenv
RUN sudo apt-get install -y ros-noetic-vision-msgs

# install rtk dependencies
RUN pip install pyzmq pyserial pyrtcm
RUN sudo usermod -a -G dialout $USER
RUN sudo apt-get install -y ros-noetic-rtcm-msgs

# MOCHA deps
RUN sudo apt-get update && sudo apt-get install -y iputils-ping default-jre 
# Scorecard deps
RUN sudo apt-get update && sudo apt-get install -y ros-noetic-vision-opencv \
 ros-noetic-vision-msgs \
 ros-noetic-cv-bridge \
 ros-noetic-wfov-camera-msgs

COPY common/MOCHA/ ws/src/MOCHA/
COPY common/dtc_msgs ws/src/dtc_msgs
COPY common/rtk-correction ws/src/rtk-correction
COPY scoring-server-submission/watchstate ws/src/MOCHA/interface_rajant/scripts/thirdParty/watchstate
COPY scoring-server-submission/scorecard_submitter ws/src/scorecard_submitter
COPY geoviz/ ws/src/geoviz
COPY basestation-launch ws/src/basestation-launch
COPY ./entrypoint.bash entrypoint.bash
RUN sudo chown $USER:$USER entrypoint.bash && chmod +x entrypoint.bash

ENV MOCHA=false
ENV RTK=false
ENV SENDER=false
ENV VIZ=false

RUN cd ~/ws \
 && catkin config --extend /opt/ros/noetic \
 && catkin build --no-status -DCMAKE_BUILD_TYPE=Release

# make life nice inside docker
RUN sudo chown $USER:$USER ~/.bashrc \
 && /bin/sh -c 'echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo "source ~/ws/devel/setup.bash" >> ~/.bashrc' \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc

ENTRYPOINT ["/home/dtc/entrypoint.bash"]
