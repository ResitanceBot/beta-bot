FROM nvidia/cuda:11.6.2-base-ubuntu20.04

SHELL ["/bin/bash", "-c"]
 
# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# [Beta Bot]: install some extra dependencies
RUN apt-get -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN apt-get -y install python3-tk
RUN apt-get -y install git

# [Beta Bot]: install and compile beta-bot repository
RUN cd ~\
 && git clone https://github.com/ResitanceBot/beta-bot.git\
 && cd beta-bot\
 && rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y\
 && source /opt/ros/noetic/setup.bash\
 && catkin_make

WORKDIR /root

