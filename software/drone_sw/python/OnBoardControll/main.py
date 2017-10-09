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
from RMQHandler.DroneConsumer import DroneAbortWorker
from RMQHandler import DroneProducer
from GPSHandler import GPSHandler

def __main__():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rmquser', nargs='?', default="drone", type=str)
    parser.add_argument('--rmqpass', nargs='?', default="drone", type=str)
    parser.add_argument('--rmqhost', nargs='?', default="drone.stefanrvo.dk", type=str)
    parser.add_argument('--mavport', nargs='?', default='/dev/ttyLP1', type=str)
    parser.add_argument('--mavbaud', nargs='?', default=57600, type=int)
    parser.add_argument('--droneid', nargs='?', default=1, type=int)
    parser.add_argument('--loglevel', nargs='?', default="INFO", type=str)
    parser.add_argument('--gpsport', nargs='?', default="/dev/ttyLP0", type=str)
    parser.add_argument('--gpsbaud', nargs='?', default=115200, type=int)



    args = parser.parse_args()

    logging.getLogger().setLevel(args.loglevel)
    logging.getLogger().addHandler(logging.StreamHandler())
    logging.getLogger().addHandler(logging.handlers.SysLogHandler(address = '/dev/log'))
    logging.info("Starting Drone Onboard Control!")

    #Setup mavlink connection
    logging.info("Establishing mavlink connection")
    drone = DroneController(port = args.mavport, baud = args.mavbaud)
    logging.info("Established mavlink connection")
    gps_handler = GPSHandler.GPSHandler(args.gpsport, args.gpsbaud)
    gps_handler.start_handler()

    #Setting up kombu connection
    with kombu.Connection("amqp://{}:{}@{}:5672/".format(args.rmquser, args.rmqpass, args.rmqhost), failover_strategy='shuffle', heartbeat=4)  as connection:
        DroneProducer.initialise(args.droneid, connection)
        worker = DroneAbortWorker(connection, args.droneid, drone)
        worker.run()


if __name__ == "__main__":
    __main__()
