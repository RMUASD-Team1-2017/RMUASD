#!/usr/bin/env python

from aed_gcs_logic.msg import waypoints
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import rospy
import time
print "Starting Mission Test"

rospy.init_node('mission_test', anonymous=True)
publisher = rospy.Publisher("/path", waypoints, queue_size=1)

path = waypoints()
waypoint1 = NavSatFix()
waypoint2 = NavSatFix()
waypoint3 = NavSatFix()
waypoint4 = NavSatFix()

# Simple route close to the runway
# Start: 55.475442, 10.330734
# First point: 55.475707, 10.331780
# Second Point: 55.476210, 10.331199
# Landing Point: 55.475836, 10.329944

# Simple route on test airfield.
# Start: 55.471377, 10.323792
# First point: 55.471800, 10.323653
# Second Point: 55.471625, 10.323037
# Landing point: 55.471246, 10.323417

waypoint1.latitude = 55.3667009
waypoint1.longitude = 10.4311024

waypoint2.latitude = 55.366078
waypoint2.longitude = 10.430959

# waypoint3.latitude = 55.476210
# waypoint3.longitude = 10.331199
#
# waypoint4.latitude = 55.47583
# waypoint4.longitude = 10.329944

path.path = [waypoint1, waypoint2]

print "Publishing"
time.sleep(1)
publisher.publish(path)
