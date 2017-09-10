#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from aed_gcs_logic.srv import *

class path_planner:
    """Path Planner class"""

    def __init__(self):
        self.mission_request_service = rospy.Service('/path_planner/request_mission', mission_request, mission_request_callback)

    def get_map_data(self):
        print "Getting map data"

    def find_path(self, data):
        print "Finding Path"

        # Find path here
        # ...

        path = "Somewhere to somewhere else"

        return path

    def mission_request_callback(self, data):
        print "Recieved mission"

        path = find_path(data)

        # Execute path
        # ...

        print "Mission completed"

if __name__ == "__main__":

    print "Starting Path Planner Node"

    rospy.init_node('path_planner_node', anonymous=True)

    planner_object = path_planner()

    planner_object.get_map_data()

    rospy.spin()
