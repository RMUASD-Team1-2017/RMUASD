#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from aed_gcs_logic.msg import *

class mission_handler:
    """Path Planner class"""

    def __init__(self):
        self.risk_metric_sub = rospy.Subscriber("/risk_assessment/risk_metric", String, self.risk_metric_callback)

        self.current_risk_metric = 0

    def risk_metric_callback(self, data):
        print "Recieved risk metric"

        self.current_risk_metric = data.data

if __name__ == "__main__":

    print "Starting Path Planner Node"

    rospy.init_node('mission_handler_node', anonymous=True)

    handler_object = mission_handler()

    rospy.spin()
