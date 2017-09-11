#!/usr/bin/env python
import rospy

from std_srvs.srv import Trigger

class docking_station_dummy:
    """Docking Station Dummy class"""

    def __init__(self):
        self.mission_request_service = rospy.Service('/docking_station/opens', Trigger, self.mission_request_callback)
        self.isOpen = False

    def open_callback(self, data):
        print "Opening Docking Station"
        Trigger response
        if self.isOpen:
            reponse.message = "Already Open"
            response.success = True
            return response
        else:
            rospy.sleep(5.)
            print "Docking Station Now Open"
            response.success = True
            response.message = "Opened Docking Station"
            return response

if __name__ == "__main__":

    print "Starting Path Planner Node"

    rospy.init_node('mission_handler_node', anonymous=True)

    docking_station = docking_station_dummy()

    rospy.spin()
