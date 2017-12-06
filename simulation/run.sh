#!/bin/bash
this_dir=$(pwd)
export PX4_HOME_LAT=55.5577531
export PX4_HOME_LON=10.1095593
export PX4_HOME_ALT=0
source /opt/ros/kinetic/setup.bash
cd $(pwd)/Firmware/
make posix_sitl_ekf2_$1 gazebo  -j4 
