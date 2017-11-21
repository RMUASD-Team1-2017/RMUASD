#!/usr/bin/env python
import os
import time
import logging
class GPIO:
    def __init__(self, pin, direction):
        self.pin = str(pin)
        self.direction = direction.lower()
        self.setup_pin()

    def setup_pin(self):
        try:
            if not self.is_exported():
                with open("/sys/class/gpio/export", 'w') as export:
                    export.write(self.pin)
            with open(self.get_pin_path() + "direction", "w") as direction:
                direction.write(self.direction)
        except IOError:
            logging.error("Could not set up the pin, are you sure that gpios works?")

    def get_pin_path(self):
        return "/sys/class/gpio/gpio{}/".format(self.pin)

    def set(self, state):
        if self.direction == "in":
            raise RuntimeError("You can't set an input pin!")
        if state.__class__ == bool:
            if state: state = "1"
            else: state = "0"
        try:
            with open(self.get_pin_path() + "value", 'w') as value:
                value.write(state)
        except IOError:
            logging.error("Could not set the pin to the wanted state!")
    def is_exported(self):
        if os.path.isdir(self.get_pin_path()): return True
        return False

def __main__():
    gpio = GPIO("46", "out")
    while True:
        gpio.set("1")
        gpio.set("0")

if __name__ == "__main__":
    __main__()
