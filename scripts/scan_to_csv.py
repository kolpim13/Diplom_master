#!/usr/bin/env python
import rospy
import sys
import json
import os.path
from pathlib import Path
import traceback
from sensor_msgs.msg import LaserScan
from math import floor

PATH_TO_FOLDER = "/home/maks/catkin_ws/src/diplom/records/test.json"

def position_topic_read():
    pass

def laser_topic_read():
    laser_topic_name = "/laser/scan"
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    distances = laser.ranges
    range_min = laser.range_min
    range_max = laser.range_max
    return (distances, range_min, range_max)

def compress_laser_data(data: list, range_min: float, range_max: float, limits: int):
    length = len(data)
    ran = range_max / limits
    out = [0 for i in range(limits)]

    for item in data:
        lim = floor(item / ran)
        out[lim] = out[lim] + 1

    return out

def write_to_file():
    (distances, range_min, range_max) = laser_topic_read()
    compressed = compress_laser_data(distances, range_min, range_max, 50)
    
    # Create a mes


    # Write to file
    with open(PATH_TO_FOLDER, 'w') as jsonfile:
        json.dump({"pos": [0, 0, 0], "dist": distances, "hist": compressed}, jsonfile)
        jsonfile.write(str('\n'))
        json.dump({"pos": [1, 1, 1], "dist": distances, "hist": compressed}, jsonfile)

if __name__ == '__main__':
    rospy.init_node('logger')
    write_to_file()