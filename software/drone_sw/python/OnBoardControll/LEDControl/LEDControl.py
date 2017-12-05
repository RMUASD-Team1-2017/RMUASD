#!/usr/bin/env python
# This class is used for controlling three ws2812b LED rings.
# They are connencted to an Arduino, which we comunicate to using I2C
# We will also use these rings for debugging, and are able to manually set the colors of the individual rings.
I2C_DEV_NUM = 0
I2C_ADDRESS = 0x08
MODE_OFF = 0x00
MODE_BLINK = 0x01
MODE_ROTATE = 0x02
OUTER_RING = 0x11
MIDDLE_RING = 0x12
INNER_RING = 0x13
MODE_DEBUG = 0x21



import serbus
import time
import threading
from gpioControl.gpioControl import GPIO
import logging
DEBUGCOLORS = { "GPS_INTERNAL_FIX" : (OUTER_RING, 'r'),
                "GPS_EXTERNAL_FIX" : (OUTER_RING, 'g'),
                "GPS_DISAGREE"     : (OUTER_RING, 'b'),
                "TELEM1"           : (MIDDLE_RING, 'r'),
                "TELEM2"           : (MIDDLE_RING, 'g'),
                "GSM"              : (INNER_RING, 'r'),
                "GCS_GSM"          : (INNER_RING, 'g'),
                "RESERVED_1"       : (INNER_RING, 'b'), #GSM PPP interface up.
              }
SIRENPIN = 46


class SirenControl:
    def __init__(self):
        self.pin = GPIO(pin = SIRENPIN, direction = "out")
    def on(self):
        self.pin.set(state = True)
    def off(self):
        self.pin.set(state = False)

class LEDControl:
    def __init__(self) :
        self.initialised = False
        self.lock = threading.RLock()
        self.siren = SirenControl(mode = "ON")

    def initialise(self, ledmode = MODE_BLINK):
        with self.lock:
            if   ledmode == "BLINK"   : ledmode = MODE_BLINK
            elif ledmode == "ROTATE"  : ledmode = MODE_ROTATE
            elif ledmode == "DEBUG"   : ledmode = MODE_DEBUG
            elif ledmode == "OFF"     : ledmode = MODE_OFF
            elif ledmode == "DUMMY" : pass

            if not ledmode in [MODE_OFF, MODE_BLINK, MODE_ROTATE, MODE_DEBUG, "DUMMY"]:
                raise Exception("An invalid mode was selected for LED_Control")
            self.ledmode = ledmode
            if not self.ledmode == "DUMMY":
                try:
                    self.i2cbus = serbus.I2CDev(I2C_DEV_NUM)
                    self.i2cbus.open()
                    self.i2cbus.write(I2C_ADDRESS, [self.ledmode, 0x00, 0x00, 0x00])
                except IOError:
                    logging.exception("Exception when trying to change LED colors")
            if ledmode in [MODE_BLINK, MODE_ROTATE]:
                self.siren.on()
            else:
                self.sirent.off()
            self.initialised = True
    def set_mode(self, mode):
        #just re-initialize, this should not do any harm
        self.initialise(ledmode = mode)

    def __readColor(self, ring):
        with self.lock:
            return self.i2cbus.readTransaction(I2C_ADDRESS, ring, 3)

    def setColor(self, ring, color): #color must be a list of [R,G,B], ring must be OUTER_RING, MIDDLE_RING or INNER_RING
        if not self.initialised or self.ledmode == "DUMMY" or not ring in [OUTER_RING, MIDDLE_RING, INNER_RING]:
            return
        with self.lock:
            try:
                self.i2cbus.write(I2C_ADDRESS, [ring] + color)
                return
            except IOError:
                logging.exception("Exception when trying to change LED colors")

    def setColorComponent(self, ring, component, value): #Change the color of a single component (e.g., r, g, b)
        if not component.lower() in ['r', 'g', 'b']:
            raise Exception("{} is not a valid color component".format(component))
        if not ring in [OUTER_RING, MIDDLE_RING, INNER_RING] or self.ledmode == "DUMMY":
            return

        if component.lower() == 'r':
            component = 0x01
        elif component.lower() == 'g':
            component = 0x02
        elif component.lower() == 'b':
            component = 0x03
        with self.lock:
            try:
                self.i2cbus.write(I2C_ADDRESS, [ring, component, value])
                return
            except IOError:
                logging.exception("Exception when trying to change LED colors")

    def setDebugColor(self, debug_type, status):
        if not debug_type in DEBUGCOLORS.keys():
            logging.error("Requested invalid debug color!")
            return
        if status:
            intensity = 255
        else:
            intensity = 0

        self.setColorComponent(DEBUGCOLORS[debug_type][0], DEBUGCOLORS[debug_type][1], intensity)

led = LEDControl()


if __name__ == "__main__":
    #Change through the colors for each ring
    led.initialise(ledmode = MODE_DEBUG, dummy = False)
    while True:
        for color in [[255, 0, 0], [0, 255, 0], [0, 0, 255],
                      [255, 255, 0], [255, 0, 255], [0, 255, 255], [255, 255, 255]]:
            for ring in [OUTER_RING, MIDDLE_RING, INNER_RING]:
                    led.setColor(ring, color)
                    time.sleep(0.1)
                    led.setColor(ring, [0,0,0])
