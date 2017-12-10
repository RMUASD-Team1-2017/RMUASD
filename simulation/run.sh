#!/bin/bash
this_dir=$(pwd)

if [[ -z "$PX4_HOME_LAT" ]]; then
	export PX4_HOME_LAT=55.5577531
fi
if [[ -z "$PX4_HOME_LON" ]]; then
	export PX4_HOME_LON=10.1095593
fi
if [[ -z "$PX4_HOME_ALT" ]]; then
	export PX4_HOME_ALT=0
fi
source /opt/ros/kinetic/setup.bash
cd $(pwd)/Firmware/
make posix_sitl_ekf2_$1 gazebo  -j4 
