from __future__ import print_function
#simply monitors that some communication is active on the given link
import serial
import threading
import time
import datetime
import logging
from LEDControl.LEDControl import led as debug_led

class SerialMonitor:
    def __init__(self, port, baud):
        self.run_thread = None
        self.last_communication = datetime.datetime.now()
        self.lock = threading.RLock()
        self.serial = serial.Serial(port, baud, timeout = 1)
        self.run_thread = threading.Thread(target = self.handler_thread)
        self.run_thread.daemon = True
        self.run_thread.start()

    def handler_thread(self):
        while True:
            try:
                time.sleep(1)
                if len(self.serial.read(1000)):
                    with self.lock:
                        self.last_communication = datetime.datetime.now()

            except:
                logging.exception("Got exception in serial monitor.")

    def get_last_communication(self):
        with self.lock:
            return self.last_communication
