#!/usr/bin/env python
import rospy
import logging
import threading
import sys

from mavros_msgs.msg import State
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from aed_gcs_logic.msg import *

class link_monitor:
    """Link Monitor class"""

    def __init__(self):
        self.mavros_heartbeat_sub = rospy.Subscriber("/mavros/state", State, self.mavros_heartbeat_callback, queue_size=1)
        self.gsm_heartbeat_sub = rospy.Subscriber("/drone/heartbeat", Empty, self.gsm_heartbeat_callback, queue_size=1)
        self.link_monitor_mavros = rospy.Publisher("/link_monitor/timeout_mavros", Bool, queue_size=1)
        self.lastHeartbeatMavros = rospy.get_rostime().secs
        self.mavrosConnectionUp = False
        self.lastHeartbeatGSM = rospy.get_rostime().secs

        self.acceptedDowntimeMavros = 20 # Seconds
        self.acceptedDowntimeGSM = 20 # Seconds

    def mavros_heartbeat_callback(self, data):
        self.lastHeartbeatMavros = rospy.get_rostime().secs


    def gsm_heartbeat_callback(self, data):
        self.lastHeartbeatGSM = rospy.get_rostime().secs

    def mavros_check(self):
            threading.Timer(1.0, self.mavros_check).start() #Call ourself in 1 second

            timeDifference = rospy.get_rostime().secs - self.lastHeartbeatMavros
            if timeDifference > self.acceptedDowntimeMavros and self.mavrosConnectionUp:
                self.link_monitor_mavros.publish(False)
                self.mavrosConnectionUp = False
            elif timeDifference < self.acceptedDowntimeMavros and not self.mavrosConnectionUp:
                self.link_monitor_mavros.publish(True)
                self.mavrosConnectionUp = True

    def start_monitor(self):
        self.mavros_check()


if __name__ == "__main__":

    print "Starting Link Monitoring Node"

    rospy.init_node('link_monitor_node', anonymous=True)

    monitor_handler = link_monitor()
    monitor_handler.start_monitor()
    rospy.spin()
