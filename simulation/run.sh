#!/bin/bash
this_dir=$(pwd)
export PX4_HOME_LAT=55.557882
export PX4_HOME_LON=10.109294
export PX4_HOME_ALT=0
source /opt/ros/kinetic/setup.bash
cd $(pwd)/Firmware/
make posix_sitl_ekf2_serial gazebo  -j4 
