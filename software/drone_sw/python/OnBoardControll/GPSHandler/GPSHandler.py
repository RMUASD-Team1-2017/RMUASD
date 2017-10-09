from __future__ import print_function

import serial
import pynmea2
import threading
import time

class GPSHandler:
    def __init__(self, port, baud):
        self.serial = serial.Serial(port, baud)
        self.run_thread = None

    def start_handler(self):
        self.run_thread = threading.Thread(target = self.handler_thread)
        self.run_thread.daemon = True
        self.run_thread.start()

    def handler_thread(self):
        while True:
            print(self.serial.readline(), end = '')
