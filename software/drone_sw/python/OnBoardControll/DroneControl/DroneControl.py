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
from GeoFenceChecker.GeoFenceChecker import fencechecker
class DroneController:
    def __init__(self, port, baud, connectstring):
        print(port, baud)
        if connectstring:
            self.vehicle = connect(connectstring, rate = 2, wait_ready = True, heartbeat_timeout = 60 * 60 * 24 * 365 * 10)
        else:
            self.vehicle = connect(port, rate = 2, baud = baud, wait_ready = True, heartbeat_timeout = 60 * 60 * 24 * 365 * 10) #Ten year timeout, we want to continue trying to reconnect no matter what
        print("Connected")
        print("Listing parameters")
        #If the used serial port actually disapears, there will be no way to recover, but this can only happen for non-usb serial ports, so it should not really matter
        print("Listed parameters")
        self.home_set = False
        self.vehicle.drone_controller = self #Hack to access this class from the vehicle in the quite strange decorator functions
        lock = threading.RLock()
        self.lock = lock
        self.location = None
        self.last_fix = None
        self.last_communication = datetime.datetime.now()

        @self.vehicle.on_message('*')
        def listener(self, name, message):
            with lock:
                self.drone_controller.last_communication = datetime.datetime.now()

        @self.vehicle.on_message('HOME_POSITION')
        def listener(self, name, home_position):
            with lock:
                self.drone_controller.home_set = True

        @self.vehicle.on_message('GPS_RAW_INT')
        def listener(self, name, message):
            #We import the publisher everytime, as it may have updated
            with lock:
                self.drone_controller.location = {"lat" : self.location.global_relative_frame.lat, "lon" : self.location.global_relative_frame.lon, "alt" : self.location.global_relative_frame.alt}
                self.drone_controller.last_fix = datetime.datetime.now()

            info = {"eph" : self.gps_0.eph, "epv" : self.gps_0.epv, "fix_type" : self.gps_0.fix_type, "satellites_visible" : self.gps_0.satellites_visible}

            logging.debug("Got onboard GPS: {}, {}".format(self.location.global_relative_frame, self.gps_0) )

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
                logging.warning("Softabort requested, but vehicle is not on a mission")
                return
            logging.warning("RTL requested!")

            self.vehicle.mode = VehicleMode("RTL")

    def hardabort(self):
        with self.lock:
            # In testing, we do not do this...
            #self.vehicle.armed = False
            logging.warning("Recieved a hard abort command, but this is not enabled ATM!!")

    def powercut(self):
        with self.lock:
            #Cut the power for the drone
            logging.warning("Recieved a power cut command, but this is not implemented ATM!!")

    def land(self):
        with self.lock:
            if not self.vehicle.mode == VehicleMode("MISSION"):
                logging.warning("Landing requested, but vehicle is not on a mission")
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
