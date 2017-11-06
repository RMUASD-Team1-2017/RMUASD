FROM ros:kinetic-robot

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /usr/src/
WORKDIR /usr/src/app/
ADD src /usr/src/app/src
ADD install_geographiclib_datasets.sh /usr/src/app/

RUN apt update && apt install -y ros-kinetic-mavlink ros-kinetic-geographic-msgs geographiclib-tools protobuf-compiler ros-kinetic-diagnostic-updater ros-kinetic-mavros-msgs python-pip
RUN wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.48.tar.gz && tar xfpz GeographicLib-1.48.tar.gz
RUN cd GeographicLib-1.48 && mkdir build && cd build && cmake .. && make -j4 && make install
RUN /bin/bash ./install_geographiclib_datasets.sh

RUN source /opt/ros/kinetic/setup.bash && catkin_make -j4 || catkin_make -j4 || catkin_make -j4
ADD requirements.txt /usr/src/app/
ADD run_with_mavros.sh /usr/src/app/
RUN pip install future
RUN pip install -r requirements.txt
EXPOSE 14540

ENTRYPOINT ["./run_with_mavros.sh"]
