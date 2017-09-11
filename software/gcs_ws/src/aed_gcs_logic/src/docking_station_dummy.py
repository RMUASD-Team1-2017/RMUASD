#!/usr/bin/env python
import rospy

from std_srvs.srv import Trigger

class docking_station_dummy:
    """Docking Station Dummy class"""

    def __init__(self):
        self.open_service = rospy.Service('/docking_station/opens', Trigger, self.open_callback)
        self.restart_service = rospy.Service('/docking_station/restart', Trigger, self.restart_callback)
        self.isOpen = False

    def open_callback(self, data):
        print "Opening Docking Station"
        if self.isOpen:
            return [True, "Docking Station Alreaedy Open"]
        else:
            rospy.sleep(5.)
            print "Docking Station Now Open"
            return [True, "Opened Docking Station"]

    def restart_callback(self, data):
        self.isOpen = False


if __name__ == "__main__":

    print "Starting Docking Station Node"

    rospy.init_node('docking_station_node', anonymous=True)

    docking_station = docking_station_dummy()

    rospy.spin()
