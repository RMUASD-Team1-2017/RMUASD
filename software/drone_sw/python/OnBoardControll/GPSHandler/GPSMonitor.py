#This class monitors the GPS info from the flight controller as well as the external GPS
#If this info does not agree to some certainty, an hardabort will be performed.
#We do not consider differences in height.

import threading
import traceback
import logging
import time
import datetime
from pyproj import Proj
import numpy
projection = Proj(init="epsg:7419") #EPSG:7416 / ETRS89 / UTM zone 32N (http://spatialreference.org/ref/epsg/7416/)
FIX_LOST_AGE = datetime.timedelta(seconds = 5)

FAILURE_DEVIATION_BASE = 15 #We never fail if gps difference is below 15 meters
FAILURE_DEVIATION_DELAY = 10 #We allow 10 extra meters of deviation per second
# All differences is betwee max and min

class GPSMonitor:
    #Class for porforming gps monitoring.
    # Monitor functions is a list of functions which return tuples of the format:
    #(severty ("FAIL", "WARNING", "IGNORE"), {"lat" : float, "lon" : float, "alt" : float},  last_fix (datetime),)
    #The severty levels indicates what we are going to do if the monitor lost it fix.
    def __init__(self, monitor_functions, abort_function):
        self.monitor_functions = monitor_functions
        self.abort_function = abort_function
        self.run_thread = None
        self.isNominal = False #This function indicates whether GPS was nominal during last check

    def isOperational(self):
        return self.isNominal

    def start_monitor(self):
        self.run_thread = threading.Thread(target = self.monitor)
        self.run_thread.daemon = True
        self.run_thread.start()

    def monitor(self):
        start_time = datetime.datetime.now()
        logging.info("GPS Monitoring starting. Waiting 10 seconds untill enforcement, to ensure that the GPS lock stabilizes")
        time.sleep(10)
        logging.info("GPS Monitor enforcement now active!")

        while True:
            time.sleep(1)
            isNominal = True
            try:
                current_time = datetime.datetime.now()
                locations = [] #List if tuples with utm, fix_time
                for severty, location, last_fix in [x() for x in self.monitor_functions]:
                    if last_fix and None not in location.values(): fix_age = current_time - last_fix
                    else: fix_age = None
                    if fix_age == None or fix_age > FIX_LOST_AGE:
                        isNominal = False
                        if severty == "FAIL":
                            logging.error("Lost needed GPS fix, aborting!")
                            self.abort_function()
                            break
                        elif severty == "WARNING":
                            logging.warning("Lost GPS fix on GPS with severty warning. Ignoring for now")
                        elif severty == "IGNORE":
                            logging.debug("Lost GPS fix on GPS with severty IGNORE")
                        continue
                    locations.append( (projection(location["lat"], location["lon"]), last_fix) )

                if len(locations) <= 1: #We can't check difference if only one point exists
                    continue
                p1, p2, distance = self.find_furthest_points(locations)
                time_diff = abs(p1[1] - p2[1])
                #Test if distance is withing limits
                if distance > FAILURE_DEVIATION_BASE + FAILURE_DEVIATION_DELAY * time_diff.total_seconds():
                    logging.error("GPS deviation was {} meters, which is outside limits. Aborting!".format(distance))
                    self.abort_function()
                    isNominal = False
            except:
                isNominal = False
                logging.exception("Got exception in GPSMonitor")
            self.isNominal = isNominal


    def find_furthest_points(self, locations):
        #Very slow algorithm to find the two farthest points. O(N^2). Will be ok for < ~10 points
        largest_diff = 0
        furthest_points = None
        for i in range(len(locations)):
            for j in range(i + 1, len(locations)):
                distance = numpy.linalg.norm( [locations[i][0][0] - locations[j][0][0], locations[i][0][1] - locations[j][0][1] ] )
                if distance >= largest_diff:
                    largest_diff = distance
                    furthest_points = [ locations[i], locations[j] ]
        return tuple(furthest_points + [largest_diff])
        #Returns the two furthest points
