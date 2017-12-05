from __future__ import print_function

from dronekit import connect, Command, LocationGlobal, VehicleMode
import logging
import time
import math
from pymavlink import mavutil
import traceback
import threading
import datetime
MAV_MODE_AUTO   = 4
BATTERY_CELLS = 3
MIN_VOLTAGE = 3500 * BATTERY_CELLS

from LEDControl.LEDControl import led as debug_led
from gpioControl.gpioControl import GPIO
LOW_MOTOR_PIN = 39 #pin 17
HIGH_MOTOR_PIN = 49 #pin 15


from GeoFenceChecker.GeoFenceChecker import fencechecker

class DroneController:
    def __init__(self, port, baud, connectstring, real_hard_abort = False):
        print(port, baud)
        if connectstring:
            self.vehicle = connect(connectstring, rate = 1, wait_ready = False, heartbeat_timeout = 60 * 60 * 24 * 365 * 10)
        else:
            self.vehicle = connect(port, rate = 1, baud = baud, wait_ready = False, heartbeat_timeout = 60 * 60 * 24 * 365 * 10) #Ten year timeout, we want to continue trying to reconnect no matter what
        self.setup_parameters()
        logging.info("Connected")
        lock = threading.RLock()
        self.lock = lock
        self.motor_enable_pins = { True : GPIO(pin = HIGH_MOTOR_PIN, direction = "out"),
                                   False : GPIO(pin = LOW_MOTOR_PIN,  direction = "out") }
        self.enable_motors()
        self.real_hard_abort = real_hard_abort
        self.home_set = False
        self.vehicle.drone_controller = self #Hack to access this class from the vehicle in the quite strange decorator functions
        self.location = None
        self.last_fix = None
        self.last_communication = datetime.datetime.utcnow()
        self.battery = 0
        self.min_battery = 10000000000

        @self.vehicle.on_message('SYS_STATUS')
        def listener(self, name, message):
            voltage = message.to_dict()["voltage_battery"]
            self.drone_controller.battery = voltage
            if self.drone_controller.battery < self.drone_controller.min_battery:
                self.drone_controller.min_battery = self.drone_controller.battery

            logging.info("Battery voltage {}, min was {}".format(voltage, self.drone_controller.min_battery))
            if voltage < MIN_VOLTAGE:
                logging.warning("Battery voltage is too low, trying to land immediately!")
                self.drone_controller.land()
            with lock:
                self.drone_controller.last_communication = datetime.datetime.utcnow()

        @self.vehicle.on_message('HOME_POSITION')
        def listener(self, name, home_position):
            with lock:
                self.drone_controller.home_set = True

        @self.vehicle.on_message('GPS_RAW_INT')
        def listener(self, name, message):
            got_fix = False
            with lock:
                self.drone_controller.location = {"lat" : self.location.global_relative_frame.lat, "lon" : self.location.global_relative_frame.lon, "alt" : self.location.global_relative_frame.alt}
                if not None in self.drone_controller.location.values():
                    got_fix = True
                    self.drone_controller.last_fix = datetime.datetime.utcnow()

            debug_led.setDebugColor(debug_type = "GPS_INTERNAL_FIX", status = got_fix)

            info = {"eph" : self.gps_0.eph, "epv" : self.gps_0.epv, "fix_type" : self.gps_0.fix_type, "satellites_visible" : self.gps_0.satellites_visible}

            logging.debug("Got onboard GPS: {}, {}".format(self.location.global_relative_frame, self.gps_0) )
            #We import the publisher everytime, as it may have updated
            from RMQHandler.DroneProducer import droneproducer
            if droneproducer:
                droneproducer.publish_onboard_gps(info, self.drone_controller.location)
            if fencechecker.initialised:
                try:
                    fencechecker.position_callback(self.drone_controller.location)
                except Exception as e:
                    logging.exception("Got exception while checking geofence.")

    def softabort(self):
        with self.lock:
            if not self.vehicle.mode == VehicleMode("MISSION"):
                logging.warning("Softabort requested, but vehicle is not on a mission. Mode was {}".format(self.vehicle.mode))
                return
            logging.warning("RTL requested!")

            self.vehicle.mode = VehicleMode("RTL")

    def hardabort(self):
        with self.lock:
            if self.real_hard_abort:
                self.disable_motors()
                self.vehicle.armed = False
                logging.warning("Motor shutdown performed!")

            else:
                logging.warning("Recieved a hard abort command, but this is not enabled ATM!!")

    def land(self):
        with self.lock:
            if not self.vehicle.mode == VehicleMode("MISSION"):
                logging.warning("Landing requested, but vehicle is not on a mission. Mode was {}".format(self.vehicle.mode))
                return
            logging.warning("Landing requested!")
            self.vehicle.mode = VehicleMode("LAND")

    def PX4setMode(self, mavMode):
        with self.lock:
            self.vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                                       mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                                       mavMode,
                                                       0, 0, 0, 0, 0, 0)
    def takeoff(self, altitude):
        with self.lock:
            if self.vehicle.home_location == None:
                logging.warning("No home position set yet, can not take off")
            else:
                self.vehicle.mode = VehicleMode("MISSION")
                cmds = self.vehicle.commands
                cmds.clear()
                wp = get_location_offset_meters(self.vehicle.home_location, 0, 0, altitude);
                cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0,
                wp.lat, wp.lon, wp.alt)
                cmds.add(cmd)
                cmds.upload()
                time.sleep(2)
                self.vehicle.armed = True

    def get_last_fix(self):
        with self.lock:
            return "FAIL", self.location, self.last_fix

    def get_last_communication(self):
        with self.lock:
            return self.last_communication

    def get_home_location(self):
        with self.lock:
            return self.vehicle.home_location

    def enable_motors(self):
        with self.lock:
            for pintype in self.motor_enable_pins.keys():
                self.motor_enable_pins[pintype].set(state = pintype)
    def disable_motors(self):
        with self.lock:
            for pintype in self.motor_enable_pins.keys():
                self.motor_enable_pins[pintype].set(state = not pintype)


    def setup_parameters(self):
        self.vehicle.parameters['NAV_DLL_ACT'] = 0
        self.vehicle.parameters["NAV_ACC_RAD"] = 7.0 #acceptence radius for waypoints
        self.vehicle.parameters["MPC_LAND_SPEED"] = 2.0 #Max land speed
        self.vehicle.parameters["MPC_XY_VEL_MAX"] = 15. #Max vertical velocity
        self.vehicle.parameters["MPC_TKO_SPEED"] = 15. #Max takeoff speed
        self.vehicle.parameters["MPC_Z_VEL_MAX_UP"] = 10. #Mav up velocity
        self.vehicle.parameters["MPC_Z_VEL_MAX_DN"] = 5. #Max down velocity

def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the
`original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)
