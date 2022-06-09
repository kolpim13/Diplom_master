#!/usr/bin/env python
import rospy
import sys
import json
import math
import time
import pathlib
import numpy as np

from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import LaserScan
#=======================================================================================#

# AUXILIARY FUNCTIONS
def laser_topic_read():
    laser_topic_name = "/laser/scan"
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    #distances = [round(item, 5) for item in laser.ranges]
    distances = np.array(laser.ranges)
    return distances

def find_neighbors(pos, step_x, step_y):
    top = (pos[0], round(pos[1] - step_y, 1), pos[2])
    right = (round(pos[0] + step_x, 1), pos[1], pos[2])
    bot = (pos[0], round(pos[1] + step_y, 1), pos[2])
    left = (round(pos[0] - step_x, 1), pos[1], pos[2])

    top_right = (round(pos[0] + step_x, 1), round(pos[1] - step_y, 1), pos[2])
    bot_right = (round(pos[0] + step_x, 1), round(pos[1] + step_y, 1), pos[2])
    bot_left = (round(pos[0] - step_x, 1), round(pos[1] + step_y, 1), pos[2])
    top_left = (round(pos[0] - step_x, 1), round(pos[1] - step_y, 1), pos[2])

    return (top, right, bot, left, top_right, bot_right, bot_left, top_left)
#=======================================================================================#

# COMPRESSION RAW DATA TO HISTOGRAMS
def compress_pos_absolute(data):
    global range_min, range_max, hist_pos_limits

    range = range_max / hist_pos_limits
    hist = np.zeros(hist_pos_limits)

    for item in data:
        item = 0.0 if item == float('-inf') else item
        item = float(range_max) if item == float('inf') else item
        item = item - 0.001 if item == range_max else item
        lim = math.floor(item / range)
        hist[lim] += 1 

    return hist

def compress_orient_minInSector(data):
    global range_min, range_max, laser_limits, hist_orient_limits

    range = range_max / hist_orient_limits
    hist = np.zeros(hist_orient_limits)

    ranges_in_sector = laser_limits / hist_orient_limits
    fr: int = 0
    to: int = 0
    for i in range(hist_orient_limits):
        to = round(ranges_in_sector * i)
        hist[i] = min(data[fr:to])
        fr = to

    return hist

# MATCH TWO HISTOGRAMS
def match_pos_absolute(hist1, hist2):
    global laser_limits
    amount = laser_limits * 2

    sum = 0
    for i in range(len(hist1)):
        sum += abs(hist2[i] - hist1[i])

    error = sum / amount
    return error
#=======================================================================================#

# Returns index of the best element in the arrays
def find_pos_lowRes(hist):
    global map_pos_low, map_loc_low, LR_i

    min_error: float = 100.0
    best_index: int = 0
    best_pos = None

    for i in range(LR_i):
        error = match_pos_absolute(hist, map_pos_low[i])
        if min_error > error:
            min_error = error
            best_index = i

    return best_index
#=======================================================================================#

# PRE PROCESSING
def read_descriptor():
    global PATH_TO_MAP_DESCRIPTOR

    with open(PATH_TO_MAP_DESCRIPTOR, 'r') as jsonfile:
        data = json.load(jsonfile)
        x_begin = float(data['x_begin'])
        y_begin = float(data['y_begin'])
        x_end = float(data['x_end'])
        y_end = float(data['y_end'])
        LR = float(data['lowRes'])
        HR = float(data['highRes'])
        LR_i = int(data['lowRes_i'])
        HR_i = int(data['highRes_i'])

    return (x_begin, y_begin, x_end, y_end, LR, HR, LR_i, HR_i)

def buffer_map(path_to_map, total_lines):
    global x_begin, y_begin, x_end, y_end
    global hist_pos_limits, hist_orient_limits, func_compress_pos 

    map_pos = np.zeros((total_lines, hist_pos_limits), dtype=np.int32)
    map_orient = np.zeros((total_lines, hist_orient_limits), dtype=np.float32)
    map_loc = np.zeros((total_lines, 3), dtype=np.float32)

    with open(path_to_map, 'r') as jsonfile:
        for index in range(int(total_lines)):
            line = jsonfile.readline()
            json_line = json.loads(line)
            raw_data = json_line["raw_data"]
            map_pos[index, :] = func_compress_pos(raw_data)
            # map_orient[index, :] = 
            map_loc[index, :] = np.array(json_line["pos"], dtype=np.float32)

    return (map_pos, map_orient, map_loc)
#=======================================================================================#

if __name__ == '__main__':
    # ROS 
    rospy.init_node("map_localization")
    rate = rospy.Rate(0.5)

    # Ros parameters read
    rospy.loginfo("Read all parameters")
    map_name = rospy.get_param("~map_name")
    gazebo_model_name = rospy.get_param("~gazebo_model_name")
    hist_pos_limits = rospy.get_param("~hist_pos_limits")
    hist_orient_limits = rospy.get_param("~hist_orient_limits")
    range_min = rospy.get_param("~laser_min")
    range_max = rospy.get_param("~laser_max")
    laser_limits = rospy.get_param("~laser_amount")
    threshold = rospy.get_param("~threshold")
    func_compress_pos = rospy.get_param("~func_compress_pos")           # Way for data to be compressed
    func_compress_orient = rospy.get_param("~func_compress_orient")
    func_match_pos = rospy.get_param("~func_match_pos")                 # Way for histograms to be matched
    func_match_orient = rospy.get_param("~func_match_orient")

    # Set relatives path 
    BASE_DIR = pathlib.Path(__file__).resolve(strict=True).parent
    PATH_TO_MAP_FOLDER = pathlib.Path(BASE_DIR.parent, "records")
    PATH_TO_MAP_DESCRIPTOR = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_description.txt")
    PATH_TO_MAP_LOW_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_lowRes.txt")
    PATH_TO_MAP_HIGH_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_highRes.txt")

    # Read parameters from file
    (x_begin, y_begin, x_end, y_end, LR, HR, LR_i, HR_i) = read_descriptor()

    # Choose compression and matching functions
        # pos compression function
    if func_compress_pos == "absolute":
        func_compress_pos = compress_pos_absolute
    if func_compress_orient == "minInSector":
        func_compress_orient = compress_orient_minInSector
    #if func_match_pos == "absolute":
    #    func_match_pos = 
    #if func_match_orient == "absolute":
    #    func_match_orient = 

    # Buffer maps
    rospy.loginfo("Buffer maps from files start")
    (map_pos_low, map_orient_low, map_loc_low) = buffer_map(PATH_TO_MAP_LOW_RES, LR_i)
    (map_pos_high, map_orient_high, map_loc_high) = buffer_map(PATH_TO_MAP_HIGH_RES, HR_i)
    rospy.loginfo("Buffer maps from files done:")
    rospy.loginfo(map_pos_low)
    rospy.loginfo(map_orient_low)
    rospy.loginfo(map_loc_low)
    rospy.loginfo("================================================================")

    # Main loop
    while not rospy.is_shutdown():
        start_time = time.time()

        # Read laser raw_data and convert them into histogram
        ranges = laser_topic_read()
        hist = func_compress_pos(ranges)
        rospy.loginfo("Raw data was read and compressed into histograms")

        # Find best candidate on the low resolution map
        best_index_low = find_pos_lowRes(hist)
        best_low = np.array(map_loc_low[best_index_low])
        rospy.loginfo("Best candidate on the low resolution map")
        rospy.loginfo("x_low = %.5f, y_high = %.5f", best_low[0], best_low[1])


        #rospy.loginfo("Nearest positions to check on the high resolution map")
        #rospy.loginfo("Bla Bla")


        #rospy.loginfo("Best candidate on the high resolution map")
        #rospy.loginfo("x_high = %.5f, y_high = %.5f", best_high[0], best_high[1])


        #rospy.loginfo("Approximated position")
        #rospy.loginfo("x_approx = %.5f, y_approx = %.5f", approx[0], approx[1])


        #rospy.loginfo("Orientation")
        #rospy.loginfo("phi = %.5f", orient)


        rospy.loginfo("This step took %.3f", (time.time() - start_time))
        rospy.loginfo("================================================================")
        rate.sleep()
#=======================================================================================#