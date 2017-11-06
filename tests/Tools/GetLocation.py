#!/bin/env python
#Return the location of the drone with the given id from the given url
from __future__ import print_function

import argparse
import requests
import sys
import json

def get_location(url, drone):
    s = requests.Session()
    request = s.get(url)
    print(request.json()[str(drone)])
    
def __main__():
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', nargs='?', type=int)
    parser.add_argument('--url', nargs='?', type=str)
    args = parser.parse_args()
    get_location(args.url, args.id)

if __name__ == "__main__":
    __main__()
