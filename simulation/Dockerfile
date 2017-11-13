#PX4 Firmware can not build without being a submodule from a valid git repo (wtf), so we need to build the docker image from here...
FROM ros:kinetic-robot

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /usr/src/
RUN apt update && apt install -y python-jinja2 gcc g++ unzip  ros-kinetic-mavlink ros-kinetic-geographic-msgs geographiclib-tools protobuf-compiler ros-kinetic-gazebo-dev ros-kinetic-diagnostic-updater ros-kinetic-mavros-msgs ros-kinetic-cv-bridge

WORKDIR /usr/src/
RUN git clone https://github.com/RMUASD-Team1-2017/Firmware.git

#PX4 gazebo build progress is retarted, so we need to do it like this...
RUN source /opt/ros/kinetic/setup.bash && cd Firmware && make all -j4
RUN source /opt/ros/kinetic/setup.bash && cd Firmware && DONT_RUN=1 make posix_sitl_ekf2_serial gazebo
RUN source /opt/ros/kinetic/setup.bash && cd Firmware && DONT_RUN=1 make posix_sitl_ekf2_slow_bat_drain gazebo
RUN source /opt/ros/kinetic/setup.bash && cd Firmware && DONT_RUN=1 make posix_sitl_ekf2 gazebo
ADD run.sh /usr/src/run.sh
RUN chmod +x /usr/src/run.sh
EXPOSE 14557
EXPOSE 14554

ENTRYPOINT ["/usr/src/run.sh"]
CMD ["serial"]
