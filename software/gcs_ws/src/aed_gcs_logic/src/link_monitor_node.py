#!/usr/bin/env python
import rospy
import logging
import threading
import sys
import time

from mavros_msgs.msg import State
from std_msgs.msg import Empty # empty message.
from std_msgs.msg import Bool
from aed_gcs_logic.msg import *
from aed_gcs_logic import srv
from aed_gcs_logic.srv import AbortRequest

#from State.msg import connected

class link_monitor:
    """Link Monitor class"""

    def __init__(self):
        self.mavros_heartbeat_sub = rospy.Subscriber("/mavros/state", State, self.mavros_heartbeat_callback, queue_size=1)
        self.gsm_heartbeat_sub = rospy.Subscriber("/drone/heartbeat", Empty, self.gsm_heartbeat_callback, queue_size=1)
        self.link_monitor_mavros = rospy.Publisher("/link_monitor/timeout_mavros", Bool, queue_size=1)
        self.link_monitor_GSM = rospy.Publisher("/link_monitor/timeout_GSM", Bool, queue_size=1)
        rospy.wait_for_service('drone_logic/abort')
        self.abort_callback = rospy.ServiceProxy("drone_logic/abort", AbortRequest)
        self.lastHeartbeatMavros = rospy.get_rostime().secs
        self.lastHeartbeatGSM = rospy.get_rostime().secs

        self.acceptedDowntimeMavros = 20 # Seconds
        self.acceptedDowntimeGSM = 20 # Seconds

    def mavros_heartbeat_callback(self, data):
        if data.connected == True :
            self.lastHeartbeatMavros = rospy.get_rostime().secs

    def gsm_heartbeat_callback(self, data):
        self.lastHeartbeatGSM = rospy.get_rostime().secs

    def mavros_check(self):
        timer_thread = threading.Timer(1.0, self.mavros_check) #Call ourself in 1 second
        timer_thread.daemon = True
        timer_thread.start()
        req = srv.AbortRequestRequest()
        req.abort_type = srv.AbortRequestRequest.TEST_SOFT_ABORT_RTL

        timeDifference = rospy.get_rostime().secs - self.lastHeartbeatMavros
        if timeDifference > self.acceptedDowntimeMavros:
            self.link_monitor_mavros.publish(True)
            print("Trying to abort due to lost MAVROS link. Result was {}".format(self.abort_callback(req)))
        elif timeDifference < self.acceptedDowntimeMavros:
            self.link_monitor_mavros.publish(False)

    def gsm_check(self):
        timer_thread = threading.Timer(1.0, self.gsm_check) #Call ourself in 1 second
        timer_thread.daemon = True
        timer_thread.start()
        req = srv.AbortRequestRequest()
        req.abort_type = srv.AbortRequestRequest.TEST_SOFT_ABORT_RTL
        timeDiff = rospy.get_rostime().secs - self.lastHeartbeatGSM
        if timeDiff > self.acceptedDowntimeGSM:
            self.link_monitor_GSM.publish(True)
            print("Trying to abort due to lost GSM link. Result was {}".format(self.abort_callback(req)))
        elif timeDiff < self.acceptedDowntimeGSM:
            self.link_monitor_GSM.publish(False)

    def start_monitor(self):
        time.sleep(1)
        self.mavros_check()
        self.gsm_check()




if __name__ == "__main__":

    print "Starting Link Monitoring Node"

    rospy.init_node('link_monitor_node', anonymous=True)

    monitor_handler = link_monitor()
    monitor_handler.start_monitor()
    rospy.spin()
