#!/bin/bash
source devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@${mavroshost:-localhost}:14557" &
sleep 2
roslaunch aed_gcs_logic gcs.launch
