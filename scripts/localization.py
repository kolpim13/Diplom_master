#!/usr/bin/env python
import rospy
import sys
import json
import math
from sensor_msgs.msg import LaserScan

# Variables
PATH_TO_FILE = "/home/maks/catkin_ws/src/diplom/records/map1.json"

# Functions
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
        item = 0 if item == float('-inf') else item
        item = range_max if item == float('inf') else item
        item = item - 0.001 if item == range_max else item
        lim = math.floor(item / ran)
        out[lim] = out[lim] + 1

    return out

def compare_histograms_absoluteValue(hist1: list, hist2: list):
    sum = 0.0
    for i in range(len(hist1)):
        sum += abs(hist2[i] - hist1[i])

    return sum

if __name__ == '__main__':
    rospy.init_node('localization')
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        aver_range = [0 for i in range(1000)]
        for i in range(20):
            (distances, range_min, range_max) = laser_topic_read()
            #for j in range(1000):
            #    aver_range[j] = aver_range[j] + distances[j]
            aver_range = [aver_range[j] + distances[j] for j in range(1000)]
            rospy.sleep(0.2)
        aver_range = [aver_range[j] / 20 for j in range(1000)]

        histogram = compress_laser_data(aver_range, range_min, range_max, 60)
        rospy.loginfo(histogram)
        #rospy.loginfo(distances)

        #pos = None
        #prob_estimation = 1.7976931348623157e+308

        #jsonfile = open(PATH_TO_FILE, 'r')
        #all_lines = jsonfile.readlines()
        #jsonfile.close()

        #for line in all_lines:
        #    json_line = json.loads(line)
        #    hist_i = json_line['hist']
        #    estim = compare_histograms_absoluteValue(histogram, hist_i)
        #    if estim < prob_estimation:
        #        prob_estimation = estim
        #        pos = json_line['pos']

        #rospy.loginfo("Current position: x = %.2f, y = %.2f, phi = %.2f", pos[0], pos[1], pos[2])
        rate.sleep()
