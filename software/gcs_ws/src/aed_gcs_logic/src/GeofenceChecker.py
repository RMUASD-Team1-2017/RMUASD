#!/usr/bin/env python
#This module is able to perform checks of whether the drone position is
# inside or outside a geofence described by a polygon. The polygon are not allowed to have any overlaping sections
import json
from shapely import geometry
import sys
import rospy
import logging
from aed_gcs_logic import srv

from aed_gcs_logic.srv import AbortRequest
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import rospkg
rospack = rospkg.RosPack()
from pyproj import Proj
projection = Proj(init="epsg:7419") #EPSG:7416 / ETRS89 / UTM zone 32N (http://spatialreference.org/ref/epsg/7416/)

RTL_DISTANCE = 1
LAND_DISTANCE = 20
HARD_ABORT_DISTANCE = 40

RTL_HEIGHT = 50
LAND_HEIGHT = 80
HARD_ABORT_HEIGHT = 100

class GeoFence:
    def __init__(self, fencefile):
        try:
            points = [ projection(x[0], x[1]) for x in json.loads(fencefile)["polygon"]]
            self.polygon = geometry.Polygon(points)
        except ValueError:
            with open(fencefile, "r") as _fence:
                points = [ projection(x[0], x[1]) for x in json.loads(_fence.read())["polygon"]]
                self.polygon = geometry.Polygon(points)
    def getDistance(self, point):
        if type(point) == type(NavSatFix()):
            return self.polygon.distance(geometry.Point(projection(point.latitude, point.longitude )))
        return self.polygon.distance(geometry.Point(projection(point[0], point[1] )))



class GeoFenceChecker:
    def __init__(self):
        rospy.init_node('link_monitor_node', anonymous=True)
        self.fence = GeoFence(rospy.get_param('/geofence_file'))
        self.position_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.position_callback, queue_size=10)
        self.altitude_sub = rospy.Subscriber("mavros/global_position/rel_alt", Float64, self.altitude_callback, queue_size=10)

        rospy.wait_for_service('drone_logic/abort')
        self.abort_callback = rospy.ServiceProxy("drone_logic/abort", AbortRequest)

    def position_callback(self, navsatfix):
        if None not in [navsatfix.latitude, navsatfix.longitude]: return #No GPS fix
        distance = self.fence.getDistance(navsatfix)
        req = srv.AbortRequestRequest()
        if distance == 0.0:
            return
        elif distance >= HARD_ABORT_DISTANCE:
            req.abort_type = srv.AbortRequestRequest.HARD_ABORT
        elif distance >= LAND_DISTANCE:
            req.abort_type = srv.AbortRequestRequest.SOFT_ABORT_LAND
        elif distance >= RTL_DISTANCE:
            req.abort_type = srv.AbortRequestRequest.TEST_SOFT_ABORT_RTL
        print("Trying to abort due to geofence breach. Result was {}. Distance from fence: {}".format(self.abort_callback(req), distance))

    def altitude_callback(self, altitude):
        req = srv.AbortRequestRequest()
        altitude = altitude.data
        if altitude < RTL_HEIGHT:
            return
        elif altitude >= HARD_ABORT_HEIGHT:
            req.abort_type = srv.AbortRequestRequest.HARD_ABORT
        elif altitude >= LAND_HEIGHT:
            req.abort_type = srv.AbortRequestRequest.SOFT_ABORT_LAND
        elif altitude >= RTL_HEIGHT:
            req.abort_type = srv.AbortRequestRequest.SOFT_ABORT_LAND
        print("Trying to abort due to altitude. Result was {}. altitude: {}".format(self.abort_callback(req), altitude))


if __name__  == "__main__":
    fencechecker = GeoFenceChecker()
    rospy.spin()
