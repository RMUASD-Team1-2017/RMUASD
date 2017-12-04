#!/usr/bin/env python
import rospy
import time

from std_srvs.srv import Trigger

class docking_station_dummy:
    """Docking Station Dummy class"""

    def __init__(self):
        self.open_service = rospy.Service('/docking_station/opens', Trigger, self.open_callback)
        self.restart_service = rospy.Service('/docking_station/restart', Trigger, self.restart_callback)
        self.isOpen = False

    def turn_servo(self, direction):                      # Function to blindly wind a servo clockwise or anticlockwise
        if self.direction == 1:                           # Choose direction = 1 for clockwise,
            print "Winding servo clockwise..."
            # wind code here
            rospy.sleep(5.)
        else:
            print "Winding servo anticlockwise..."
            # wind code here
            rospy.sleep(5.)

    def open_callback(self, data):
        ##############
        simulation = 0                         # If simulating docking station, set to 0. If using actual docking station, set to 1
        ##############
        if simulation  == 0:                   # dummy loop to simulate using a docking station
            print "Opening Docking Station! (--- SIMULATION MESSAGE ---)"
            if self.isOpen:
                return [True, "Docking Station is already open! (--- SIMULATION MESSAGE ---)"]
            else:
                print "... Opening ... (--- SIMULATION MESSAGE ---)"
                rospy.sleep(5.)
                print "Docking station is now open (--- SIMULATION MESSAGE ---)"
                return [True, "Opened docking station (--- SIMULATION MESSAGE ---)"]

        else:                              # If using the real docking station
            if self.isOpen:
                return [True, "Docking station is already open! (--- SIMULATION MESSAGE ---)"]
            else:
                print "... Opening ... "
                # put code in here to actually open the docking station
                # while pressure switch1 = off:
                self.turn_servo(1) # 1 => clockwise
                # set self.isOpen = True
                print "Docking station now open"
                return [True, "Opened docking station"]





    def restart_callback(self, data):
        self.isOpen = False


if __name__ == "__main__":

    print "Starting Docking Station Node"

    rospy.init_node('docking_station_node', anonymous=True)

    docking_station = docking_station_dummy()

    rospy.spin()
