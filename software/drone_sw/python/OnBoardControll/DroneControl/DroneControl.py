from dronekit import connect, Command, LocationGlobal, VehicleMode
import logging

class DroneController:
    def __init__(self, port, baud):
        print(port, baud)
        self.vehicle = connect(port, baud = baud, wait_ready = True)

        @self.vehicle.on_message('HOME_POSITION')
        def listener(self, name, home_position):
            print("HOME POSITION")
            global home_position_set
            home_position_set = True

    def softabort(self):
        self.vehicle.mode = VehicleMode("RTL")

    def hardabort(self):
        self.vehicle.armed = False

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")




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
