FROM ros:kinetic-robot

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /usr/src/
ADD . /usr/src/app/
WORKDIR /usr/src/app/
RUN apt update && apt install -y ros-kinetic-mavlink ros-kinetic-geographic-msgs geographiclib-tools protobuf-compiler ros-kinetic-diagnostic-updater
RUN wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.48.tar.gz && tar xfpz GeographicLib-1.48.tar.gz
RUN cd GeographicLib-1.48 && mkdir build && cd build && cmake .. && make -j4 && make install
RUN /bin/bash ./install_geographiclib_datasets.sh

RUN source /opt/ros/kinetic/setup.bash && catkin_make
