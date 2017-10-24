################################################################################################
# @File DroneKitPX4.py
# Example usage of DroneKit with PX4
#
# Code partly based on DroneKit (c) Copyright 2015-2016, 3D Robotics.
################################################################################################

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import time, sys, argparse, math
import numpy

################################################################################################
# Settings
################################################################################################
connection_string       = '/dev/ttyLP1'
MAV_MODE_AUTO   = 4
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py


# Parse connection argument
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect


################################################################################################
# Init
################################################################################################

# Connect to the Vehicle
print "Connecting"
vehicle = connect(connection_string, baud=57600, wait_ready=True)
#vehicle = connect('0.0.0.0:14550', wait_ready=True)



def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)



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





################################################################################################
# Listeners
################################################################################################

home_position_set = False

#Create a message listener for home position fix

#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True

################################################################################################
# Start mission example
################################################################################################

# wait for a home position lock
while not home_position_set:
    print "Waiting for home position..."
    time.sleep(1)

# Display basic vehicle state
print " Type: %s" % vehicle._vehicle_type
print " Armed: %s" % vehicle.armed
print " System status: %s" % vehicle.system_status.state
print " GPS: %s" % vehicle.gps_0
print " Alt: %s" % vehicle.location.global_relative_frame.alt

MAV_CMD_CONDITION_CHANGE_ALT = 113
print(vehicle.parameters["NAV_ACC_RAD"])

vehicle.parameters["NAV_ACC_RAD"] = 7.0 #acceptence radius for waypoints
vehicle.parameters["MPC_LAND_SPEED"] = 3.0 #Max land speed
vehicle.parameters["MPC_XY_VEL_MAX"] = 20. #Max vertical velocity
vehicle.parameters["MPC_TKO_SPEED"] = 20. #Max takeoff speed
vehicle.parameters["MPC_Z_VEL_MAX_UP"] = 10. #Mav up velocity
vehicle.parameters["MPC_Z_VEL_MAX_DN"] = 5. #Max down velocity
# Change to AUTO mode
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)

# Load commands
cmds = vehicle.commands
cmds.clear()

home = vehicle.home_location
print(home)
vehicle.mode = VehicleMode("MISSION")
cmds = vehicle.commands
cmds.clear()

wp = get_location_offset_meters(vehicle.home_location, 0, 0, 30);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0,
wp.lat, wp.lon, wp.alt)
cmds.add(cmd)
cmds.upload()
time.sleep(2)
vehicle.armed = True
time.sleep(20)


# Disarm vehicle
vehicle.armed = False
time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()
time.sleep(1)
