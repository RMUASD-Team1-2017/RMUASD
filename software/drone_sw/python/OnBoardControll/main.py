#Python service running on the drone can do soft (RTL) and hard (motor shutdown) abort,
#And send requested telemetry
from __future__ import print_function

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
def __main__():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rmquser', nargs='?', default="drone", type=str)
    parser.add_argument('--rmqpass', nargs='?', default="drone", type=str)
    parser.add_argument('--rmqhost', nargs='?', default="drone.stefanrvo.dk", type=str)
    parser.add_argument('--mavport', nargs='?', default='/dev/ttyLP2', type=str)
    parser.add_argument('--mavbaud', nargs='?', default=921600, type=int)
    parser.add_argument('--droneid', nargs='?', default=1, type=int)
    parser.add_argument('--loglevel', nargs='?', default="INFO", type=str)
    parser.add_argument('--gpsport', nargs='?', default="/dev/ttyACM0", type=str)
    parser.add_argument('--gpsbaud', nargs='?', default=115200, type=int)
    parser.add_argument("--ignoregps", action='store_true')
    parser.add_argument('--simufile', nargs='?', default=None, type=str)
    parser.add_argument('--gcsport', nargs='?', default="/dev/ttyLP1", type=str)
    parser.add_argument('--gcsbaud', nargs='?', default=57600, type=int)
    args = parser.parse_args()

    logging.getLogger().setLevel(args.loglevel)
    logging.getLogger().addHandler(logging.StreamHandler())
    logging.getLogger().addHandler(logging.handlers.SysLogHandler(address = '/dev/log'))
    logging.info("Starting Drone Onboard Control!")

    #Setup mavlink connection
    logging.info("Establishing mavlink connection")
    drone = DroneController(port = args.mavport, baud = args.mavbaud)
    logging.info("Established mavlink connection")
    gps_handler = GPSHandler(args.gpsport, args.gpsbaud, args.simufile)
    gps_handler.start_handler()
    if not args.ignoregps:
        gps_monitor = GPSMonitor([drone.get_last_fix, gps_handler.get_last_fix], drone.softabort)
        gps_monitor.start_monitor()
    connection_monitor = ConnectionMonitor( gcs_network_check = DroneConsumer.get_last_gcs_heartbeat,
                                            network_check = network_monitor.get_last_connection,
                                            telemetry_gcs_check = SerialMonitor(args.gcsport, args.gcsbaud).get_last_communication,
                                            telemetry_oes_check =  drone.get_last_communication,
                                            drone = drone
                                            )

    #Setting up kombu connection
    with kombu.Connection("amqp://{}:{}@{}:5672/".format(args.rmquser, args.rmqpass, args.rmqhost), failover_strategy='shuffle', heartbeat=4)  as connection:
        DroneProducer.initialise(args.droneid, connection)
        DroneConsumer.initialise(connection, args.droneid, drone)
        DroneConsumer.droneconsumer.run()

if __name__ == "__main__":
    __main__()
