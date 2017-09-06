FROM ros:kinetic-ros-base

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /usr/src/
ADD . /usr/src/app/
WORKDIR /usr/src/app/

RUN source /opt/ros/kinetic/setup.bash && catkin_make
