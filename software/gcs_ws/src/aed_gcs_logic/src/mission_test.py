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

#55.471434, 10.323979
#55.471683 10.323753

#55.471680 y: 10.324129
#55.471668 y: 10.323848

waypoint1.latitude = 55.4717077
waypoint1.longitude = 10.324197

#latitude: 55.4717042
#longitude: 10.3238927

waypoint2.latitude = 55.4717042
waypoint2.longitude = 10.3238474

path.path = [waypoint1, waypoint2]

print "Publishing"
time.sleep(1)
publisher.publish(path)
