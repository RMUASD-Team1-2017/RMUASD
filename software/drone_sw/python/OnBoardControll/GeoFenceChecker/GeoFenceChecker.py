#!/usr/bin/env python
#This module is able to perform checks of whether the drone position is
# inside or outside a geofence described by a polygon. The polygon are not allowed to have any overlaping sections
import json
from shapely import geometry
import sys
import logging
from pyproj import Proj
projection = Proj(init="epsg:7419") #EPSG:7416 / ETRS89 / UTM zone 32N (http://spatialreference.org/ref/epsg/7416/)
from threading import RLock
LAND_DISTANCE = 1
HARD_ABORT_DISTANCE = 100
LAND_HEIGHT = 100
HARD_ABORT_HEIGHT = 150

class GeoFence:
    def __init__(self, fencefile):
        if fencefile.__class__ == dict:
            points = [ projection(x[0], x[1]) for x in fencefile["polygon"]]
            self.polygon = geometry.Polygon(points)
        else:
            try:
                points = [ projection(x[0], x[1]) for x in json.loads(fencefile)["polygon"]]
                self.polygon = geometry.Polygon(points)
            except ValueError:
                with open(fencefile, "r") as _fence:
                    points = [ projection(x[0], x[1]) for x in json.loads(_fence.read())["polygon"]]
                    self.polygon = geometry.Polygon(points)
    def getDistance(self, point):
        if point.__class__ == dict:
            return self.polygon.distance(geometry.Point(projection(point["lat"], point["lon"] )))
        return self.polygon.distance(geometry.Point(projection(point[0], point[1] )))



class GeoFenceChecker:
    def __init__(self):
        self.initialised = False
        self.within_fence = False
        self.lock = RLock()

    def getStatus(self):
        return self.within_fence

    def intialize(self, drone, fence):
        with self.lock:
            self.fence = GeoFence(fence)
            self.drone = drone
            self.initialised = True
        logging.info("Geofence intialised")

    def position_callback(self, gpsposition):
        if not self.initialised: return False
        if None in gpsposition.values():
            logging.warning("NO GPS fix, skipping geofence check")
            return #No GPS fix
        with self.lock:
            distance = self.fence.getDistance(gpsposition)
            logging.debug("Geofence distance is {}".format(distance))
            altitude = gpsposition["alt"]
            if distance == 0.0 and altitude < LAND_HEIGHT:
                self.within_fence = True
                return

            elif distance >= HARD_ABORT_DISTANCE or altitude >= HARD_ABORT_HEIGHT:
                self.drone.hardabort()
            elif distance >= LAND_DISTANCE or altitude >= LAND_HEIGHT:
                self.drone.land()
            self.within_fence = False


        logging.warning("Trying to abort due to geofence breach. Result was Distance from fence: {}, altitude {}".format(distance, altitude))



fencechecker = GeoFenceChecker()

if __name__  == "__main__":
    fencechecker = GeoFenceChecker()
