from dronekit import connect, Command, LocationGlobal, VehicleMode
import logging
import time
import math
MAV_MODE_AUTO   = 4
from pymavlink import mavutil

class DroneController:
    def __init__(self, port, baud):
        print(port, baud)
        self.vehicle = connect(port, baud = baud, wait_ready = True)
        self.home_set = False
        self.vehicle.drone_controller = self

        @self.vehicle.on_message('HOME_POSITION')
        def listener(self, name, home_position):
            self.drone_controller.home_set = True

    def softabort(self):
        self.vehicle.mode = VehicleMode("RTL")

    def hardabort(self):
        self.vehicle.armed = False

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")

    def PX4setMode(self, mavMode):
        self.vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                                   mavMode,
                                                   0, 0, 0, 0, 0, 0)
    def takeoff(self, altitude):
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
