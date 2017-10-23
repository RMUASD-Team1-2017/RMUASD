#!/usr/bin/env python
import rospy
import logging
import threading
import sys

from mavros_msgs.msg import State
from std_msgs.msg import Empty # empty message. 
from std_msgs.msg import Bool
from aed_gcs_logic.msg import *
#from State.msg import connected

class link_monitor:
    """Link Monitor class"""

    def __init__(self):
        self.mavros_heartbeat_sub = rospy.Subscriber("/mavros/state", State, self.mavros_heartbeat_callback, queue_size=1)
        self.gsm_heartbeat_sub = rospy.Subscriber("/drone/heartbeat", Empty, self.gsm_heartbeat_callback, queue_size=1)
        self.link_monitor_mavros = rospy.Publisher("/link_monitor/timeout_mavros", Bool, queue_size=1)
        self.link_monitor_GSM = rospy.Publisher("/link_monitor/timeout_GSM", Bool, queue_size=1)
        
        self.lastHeartbeatMavros = rospy.get_rostime().secs
        self.connected = False 
        self.mavrosConnectionUp = False
        self.GSMConnectionUp = False
        self.lastHeartbeatGSM = rospy.get_rostime().secs
        
        self.acceptedDowntimeMavros = 20 # Seconds
        self.acceptedDowntimeGSM = 20 # Seconds

    def mavros_heartbeat_callback(self, data):
        self.lastHeartbeatMavros = rospy.get_rostime().secs
        self.connected = data.connected


    def gsm_heartbeat_callback(self, data):
        self.lastHeartbeatGSM = rospy.get_rostime().secs

    def mavros_check(self):
            threading.Timer(1.0, self.mavros_check).start() #Call ourself in 1 second
            
            timeDifference = rospy.get_rostime().secs - self.lastHeartbeatMavros
            print" connected ", self.connected
            #self.mavros_heartbeat_sub.connected
            if timeDifference > self.acceptedDowntimeMavros and self.mavrosConnectionUp:
                self.link_monitor_mavros.publish(False)
                self.mavrosConnectionUp = False
                print " Mavros is not accepted "
            elif timeDifference < self.acceptedDowntimeMavros and not self.mavrosConnectionUp:
                self.link_monitor_mavros.publish(True)
                self.mavrosConnectionUp = True
                print " mavros accepted "
            elif self.connected == False:
                print "lost connection "

    

    def gsm_check(self):
        threading.Timer(1.0, self.gsm_check).start() #Call ourself in 1 second
        timeDiff = rospy.get_rostime().secs - self.lastHeartbeatGSM
       # print" time", timeDiff
        if timeDiff > self.acceptedDowntimeGSM and self.GSMConnectionUp:
            self.link_monitor_GSM.publish(False)
            self.GSMConnectionUp = False
            print " GSM is not connected "
        elif timeDiff < self.acceptedDowntimeGSM and not self.GSMConnectionUp:
            self.link_monitor_GSM.publish(True)
            self.GSMConnectionUp = True
            print " GSM connected "

    def start_monitor(self):
        self.mavros_check()
        self.gsm_check()
    



if __name__ == "__main__":

    print "Starting Link Monitoring Node"

    rospy.init_node('link_monitor_node', anonymous=True)

    monitor_handler = link_monitor()
   # while(True):
    monitor_handler.start_monitor()
    rospy.spin()
