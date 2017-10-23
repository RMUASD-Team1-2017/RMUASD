from __future__ import print_function

import serial
import pynmea2
import threading
import time
import datetime
import logging

class GPSHandler:
    def __init__(self, port, baud, simulation_filepath = None):
        self.run_thread = None
        self.last_fix = None
        self.location = None
        self.lock = threading.RLock()
        print(port, baud, simulation_filepath)
        if simulation_filepath:
            self.simulation_file = open(simulation_filepath, "r")
            self.line_func = self.file_get_line
        else:
            self.serial = serial.Serial(port, baud, timeout = None)
            self.line_func = self.serial_get_line

    def start_handler(self):
        self.run_thread = threading.Thread(target = self.handler_thread)
        self.run_thread.daemon = True
        self.run_thread.start()

    def handler_thread(self):
        while True:
            try:
                line = self.line_func()
                if "$GPGGA" in line:
                    msg = pynmea2.parse(line)
                    if msg.is_valid:
                        with self.lock:
                            self.last_fix = datetime.datetime.now()
                            self.location =  {"lat" : msg.latitude, "lon" : msg.longitude, "alt" : msg.altitude}
                            logging.debug("Got external GPS location: {}".format(self.location))

                elif "$GPGSA" in line:
                    msg = pynmea2.parse(line)
                    if msg.is_valid:
                        sattelites = msg.data[2]
                        if sattelites == " ": sattelites = 12
                        info = {"eph" : float(msg.data[4]), "epv" : float(msg.data[5]), "fix_type" : int(msg.data[1]), "satellites_visible" : int(sattelites)}
                        logging.debug("Got External GPS sattelite info: {}".format(info))
                        from RMQHandler.DroneProducer import droneproducer
                        droneproducer.publish_external_gps(info = info, location = self.location)
            except:
                logging.exception("Got exception in GPS Handler")
    def serial_get_line(self):
        return self.serial.readline()


    def file_get_line(self):
        time.sleep(0.1)
        line = self.simulation_file.readline()
        if line == "":
            self.simulation_file.seek(0)
            line = self.simulation_file.readline()
        return line

    def get_last_fix(self):
        with self.lock:
            return "WARNING", self.location, self.last_fix
