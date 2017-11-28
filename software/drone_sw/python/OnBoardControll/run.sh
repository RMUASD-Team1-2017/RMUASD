#!/bin/bash
set -e
env
sleep 10
socat -d -d  udp:${simulation_ip}:14554,bind=:14541 pty,link=/dev/PX4,rawer,b921600 &
sleep 2
python main.py --rmquser drone --rmqpass drone --rmqhost ${webui_ip} --loglevel DEBUG --ignoregps --mavport /dev/PX4 --syslog 0 --ledmode DUMMY --real_hard_abort
