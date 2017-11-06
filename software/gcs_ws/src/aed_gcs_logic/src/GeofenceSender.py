#!/usr/bin/env python
#This module sends the geofence to the drone when it is requested
import sys
import rospy
import logging
import rospkg
rospack = rospkg.RosPack()
from std_msgs.msg import Empty, String

class GeoFenceSender:
    def __init__(self):
        rospy.init_node('geofence_sender', anonymous=True)
        with open(rospack.get_path("aed_gcs_logic") + "/resources/polygon_fence.fence", "r") as fencefile:
            self.fence_str = fencefile.read()
        self.fence_publisher = rospy.Publisher("drone/setgeofence", String, queue_size=1)
        self.request_sub = rospy.Subscriber("drone/requestgeofence", Empty, self.fencerequestcallback)

    def fencerequestcallback(self, *args, **kwargs):
        print("Sending geofence to drone")
        self.fence_publisher.publish(self.fence_str)





if __name__  == "__main__":
    fenceSender = GeoFenceSender()
    rospy.spin()
