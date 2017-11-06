#!/bin/env python
#Monitors the mission and returns an error if it did not finish with a specified time
from __future__ import print_function

import argparse
import requests
import sys
import json
from pyproj import Proj
import datetime
import time
import numpy
import progressbar
projection = Proj(init="epsg:7419") #EPSG:7416 / ETRS89 / UTM zone 32N (http://spatialreference.org/ref/epsg/7416/)

def monitor_progress(url, drone, goal_precision, goal_height, deadline):
    start_location = get_location(url, drone)
    bar = None
    while datetime.datetime.now() < deadline:
        if None in start_location.values():
            print("Trying to get start location, currently {}".format(start_location))
            start_location = get_location(url, drone)
        else:
            current_location = get_location(url, drone)
            goal_location = get_goal(url, drone)
            goal_coord = projection(goal_location["latitude"], goal_location["longitude"])
            current_coord =  projection(current_location["latitude"], current_location["longitude"])
            start_coord = projection(start_location["latitude"], start_location["longitude"])
            start_distance = numpy.linalg.norm( (goal_coord[0] - start_coord[0], goal_coord[1]- start_coord[1]) )
            current_distance = numpy.linalg.norm( (goal_coord[0] - current_coord[0], goal_coord[1]- current_coord[1]) )
            if bar is None:
                bar = progressbar.ProgressBar(redirect_stdout=True, max_value = start_distance, unit = "m")
            try:
                bar.update(start_distance - current_distance)
            except ValueError:
                pass
            if current_distance < goal_precision and current_location["altitude"] - start_location["altitude"] < goal_height:
                if not bar.value == bar.max_value: bar.finish()
                print("Reached goal")
                sys.exit(0)
            if current_distance < goal_precision:
                if not bar.value == bar.max_value: bar.finish()
                print("At goal position, waiting for landing. Current altitude: {} (start altitude {})".format(current_location["altitude"], start_location["altitude"]))
        time.sleep(1)
    sys.exit(1)


def get_location(url, drone):
    s = requests.Session()
    request = s.get(url)
    return request.json()[str(drone)]["position"]

def get_goal(url, drone):
    s = requests.Session()
    request = s.get(url)
    return request.json()[str(drone)]["goal"]

def __main__():
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', nargs='?', type=int)
    parser.add_argument('--locationurl', nargs='?', type=str)
    parser.add_argument('--goal_precision', nargs='?', type=float) #How close do we need to be to the goal in meters
    parser.add_argument('--goal_height', nargs='?', type=float) #How high does the drone need to be to be "landed"
    parser.add_argument('--max_mission_time', nargs='?', type=float) #The maximum time in seconds the mission is allowed to take before we report it as failed.




    args = parser.parse_args()
    end_time = datetime.datetime.now() + datetime.timedelta(seconds = args.max_mission_time)
    monitor_progress(args.locationurl, args.id, args.goal_precision, args.goal_height, end_time)

if __name__ == "__main__":
    __main__()
