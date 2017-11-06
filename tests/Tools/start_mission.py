#!/bin/env python
#This application is able to launch a mission from the webinterface
from __future__ import print_function

import argparse
import requests
import sys
import json
def requestMission(url, lat, lon):
    s = requests.Session()
    request = s.get(url, data = "")
    if request.status_code is not 200: return -1
    s.headers.update({'referer': url})
    request = s.post(url, data = {"csrfmiddlewaretoken" : request.cookies["csrftoken"], "destination_0" : lat, "destination_1" : lon, "destination_submit" : ""})
    return int(request.url.split('/')[-2])

def startMission(url, mission_id):
    s = requests.Session()
    request = s.get(url, data = "")
    if request.status_code is not 200: return -1
    s.headers.update({'referer': url, "X-CSRFToken" : request.cookies["csrftoken"], 'content-type': 'application/json'})
    request = s.post(url, data = json.dumps({"decision" : "accept", "mission" : mission_id}))
    if not request.status_code in (200, 204):
        return 1
    return 0
def __main__():
    parser = argparse.ArgumentParser()
    parser.add_argument('--lat', nargs='?', type=float)
    parser.add_argument('--lon', nargs='?', type=float)
    parser.add_argument('--requesturl', nargs='?', type=str)
    parser.add_argument('--accepturl', nargs='?', type=str)
    args = parser.parse_args()
    mission_id = requestMission(args.requesturl, args.lat, args.lon)
    if mission_id < 0:
        sys.exit(1)
    print(mission_id)
    sys.exit(startMission(args.accepturl, mission_id))

if __name__ == "__main__":
    __main__()
