#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from aed_gcs_logic.srv import *
from aed_gcs_logic.msg import *
from sensor_msgs.msg import NavSatFix

class path_planner:
    """Path Planner class"""

    def __init__(self):
        self.mission_request_service = rospy.Service('/path_planner/request_mission', mission_request, self.mission_request_callback)

    def get_map_data(self):
        print "Getting map data"

    def find_path(self, data):
        print "Finding Path"

        # Find path here
        # ...

        path = "Somewhere to somewhere else"

        return path

    def mission_request_callback(self, req):
        print "Recieved mission"

        path = self.find_path(req)

        # Execute path
        # ...

        print "Mission completed"
        return True

if __name__ == "__main__":

    print "Starting Path Planner Node"

    rospy.init_node('path_planner_node', anonymous=True)

    nav_sat_msg = NavSatFix()
    nav_sat_msg.latitude = 10.40
    nav_sat_msg.longitude = 10.50

    waypoint_msg = waypoints()
    waypoint_msg.path = [waypoint_msg, waypoint_msg, waypoint_msg]

    pub = rospy.Publisher('drone_logic/mission_path', waypoints, queue_size=10)

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        pub.publish(waypoint_msg)
        rate.sleep()

    # planner_object = path_planner()
    #
    # planner_object.get_map_data()

    rospy.spin()
