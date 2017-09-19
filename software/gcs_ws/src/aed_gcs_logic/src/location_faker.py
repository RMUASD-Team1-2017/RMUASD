#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
import rospy
import random
import time
print "Starting location faker"

rospy.init_node('location_faker', anonymous=True)
publisher = rospy.Publisher("drone/position", NavSatFix, queue_size=10)
while True:
    fix = NavSatFix()
    fix.latitude = 55.1 + random.randint(1, 10000) / 30000.
    fix.longitude = 10.4 + random.randint(1, 10000) / 30000.
    time.sleep(1)
    publisher.publish(fix)
