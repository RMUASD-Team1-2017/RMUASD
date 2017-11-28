#!/usr/bin/env python
#Python service running on the drone can do soft (RTL) and hard (motor shutdown) abort,
#And send requested telemetry

from __future__ import print_function
##Profiling start
# import yappi
# import atexit
# yappi.set_clock_type('wall')
# yappi.start()
# @atexit.register
# def finish_yappi():
#     print('[YAPPI STOP]')
#     yappi.stop()
#
#     print('[YAPPI WRITE]')
#     stats = yappi.get_func_stats()
#     for stat_type in ['pstat', 'callgrind', 'ystat']:
#       print('writing /tmp/pants.{}'.format(stat_type))
#       stats.save('/tmp/pants.{}'.format(stat_type), type=stat_type)
#
#     print('\n[YAPPI FUNC_STATS]')
#     print('writing /tmp/pants.func_stats')
#     with open('/tmp/pants.func_stats', 'wb') as fh:
#       stats.print_all(out=fh)
#
#     print('\n[YAPPI THREAD_STATS]')
#     print('writing /tmp/pants.thread_stats')
#     tstats = yappi.get_thread_stats()
#     with open('/tmp/pants.thread_stats', 'wb') as fh:
#       tstats.print_all(out=fh)

##profiling end


import kombu

import logging
import sys
import kombu
from kombu.utils import debug
import argparse
from dronekit import connect, Command, LocationGlobal, VehicleMode
from DroneControl.DroneControl import DroneController
from RMQHandler import DroneConsumer
from RMQHandler import DroneProducer
from GPSHandler.GPSHandler import GPSHandler
from GPSHandler.GPSMonitor import GPSMonitor
from NetworkMonitor.NetworkMonitor import monitor as network_monitor
from NetworkMonitor.ConnectionMonitor import ConnectionMonitor
import datetime
from SerialMonitor.SerialMonitor import SerialMonitor
from LEDControl.LEDControl import led as debug_led

def __main__():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rmquser', nargs='?', default="drone", type=str)
    parser.add_argument('--rmqpass', nargs='?', default="drone", type=str)
    parser.add_argument('--rmqhost', nargs='?', default="drone.stefanrvo.dk", type=str)
    parser.add_argument('--mavport', nargs='?', default='/dev/ttyLP2', type=str)
    parser.add_argument('--mavbaud', nargs='?', default=115200, type=int)
    parser.add_argument('--droneid', nargs='?', default=1, type=int)
    parser.add_argument('--loglevel', nargs='?', default="INFO", type=str)
    parser.add_argument('--gpsport', nargs='?', default="/dev/ttyGPS", type=str)
    parser.add_argument('--gpsbaud', nargs='?', default=115200, type=int)
    parser.add_argument("--ignoregps", action='store_true')
    parser.add_argument('--simufile', nargs='?', default=None, type=str)
    parser.add_argument('--connectstring', nargs='?', default="", type=str)
    parser.add_argument('--syslog', nargs='?', default=1, type=int)
    parser.add_argument('--ledmode', nargs='?', default="BLINK", type=str)
    parser.add_argument('--real_hard_abort', action='store_true')
    args = parser.parse_args()
    debug_led.initialise(args.ledmode)

    logging.getLogger().setLevel(args.loglevel)
    logging.getLogger().addHandler(logging.StreamHandler())
    if args.syslog:
        logging.getLogger().addHandler(logging.handlers.SysLogHandler(address = '/dev/log'))
    logging.info("Starting Drone Onboard Control!")

    #Setup mavlink connection
    logging.info("Establishing mavlink connection")
    drone = DroneController(port = args.mavport, baud = args.mavbaud, connectstring = args.connectstring, real_hard_abort = args.real_hard_abort)
    logging.info("Established mavlink connection")
    if not args.ignoregps:
        gps_handler = GPSHandler(args.gpsport, args.gpsbaud, args.simufile)
        gps_handler.start_handler()
        gps_monitor = GPSMonitor([drone.get_last_fix, gps_handler.get_last_fix], drone.land)
        gps_monitor.start_monitor()
    else: #We fake and use the onboard gps two times.
        gps_monitor = GPSMonitor([drone.get_last_fix, drone.get_last_fix], drone.land)
        gps_monitor.start_monitor()


    connection_monitor = ConnectionMonitor( gcs_network_check = DroneConsumer.get_last_gcs_heartbeat,
                                            network_check = network_monitor.get_last_connection,
                                            telemetry_oes_check =  drone.get_last_communication,
                                            drone = drone
                                            )

    #Setting up kombu connection
    with kombu.Connection("amqp://{}:{}@{}:5672/".format(args.rmquser, args.rmqpass, args.rmqhost), failover_strategy='shuffle', heartbeat=4)  as connection:
        DroneProducer.initialise(args.droneid, connection)
        DroneConsumer.initialise(connection, args.droneid, drone, gps_monitor, connection_monitor)
        DroneConsumer.droneconsumer.run()

if __name__ == "__main__":
    __main__()
