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
    distances = np.array(laser.ranges)
    return distances

#def find_neighbors(highRes_index):
#    top = 
#    right = 
#    bot = 
#    left = 
#    top_right = 
#    bot_right = 
#    bot_left =
#    top_left =

#    return (top, right, bot, left, top_right, bot_right, bot_left, top_left)
#=======================================================================================#

# COMPRESSION RAW DATA TO HISTOGRAMS
def compress_pos_absolute(data):
    global range_min, range_max, hist_pos_limits

    range = range_max / hist_pos_limits
    hist = np.zeros(hist_pos_limits)

    data = raw_data_preprocessing(data)
    for item in data:
        lim = math.floor(item / range)
        hist[lim] += 1 

    return hist

def compress_orient_minInSector(data):
    global range_min, range_max, laser_limits, hist_orient_limits

    range = range_max / hist_orient_limits
    hist = np.zeros(hist_orient_limits)

    ranges_in_sector = int(laser_limits / hist_orient_limits)
    fr: int = 0
    to: int = 0
    for i in range(hist_orient_limits):
        to = round(ranges_in_sector * i)
        hist[i] = min(data[fr:to])
        fr = to

    return hist

def compress_orient_average(data):
    global laser_limits, hist_orient_limits

    hist = np.zeros(hist_orient_limits)
    ranges_in_sector = int(laser_limits / hist_orient_limits)
    
    data = raw_data_preprocessing(data)
    for i in range(hist_orient_limits):
        index = int(i * ranges_in_sector)
        aver = 0.0
        for j in range(ranges_in_sector):
            aver += data[index + j]
        aver /= ranges_in_sector
        hist[i] = aver

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

# SOLID POINT POS AND ORIENT SEARCHING
# Returns index of the best element in the arrays
def find_pos_lowRes(hist):
    global map_pos_low, map_loc_low, LR_i
    global func_match_pos

    min_error: float = 100.0
    best_index: int = 0

    for i in range(LR_i):
        error = func_match_pos(hist, map_pos_low[i])
        if min_error > error:
            min_error = error
            best_index = i

    return best_index

# Returns aprroximated index that corresponds on the highRes map to the lowRes point
def find_corresponding_point(lowRes_index):
    global lowRes_x, lowRes_y, highRes_x, highRes_y, k

    y_low = int(lowRes_index / lowRes_y)
    x_low = int(lowRes_index % lowRes_x)
    y_high = int(y_low * k)
    x_high = int(x_low * k)
    highRes_index = y_high * highRes_y + x_high

    return highRes_index

# Give a region og points to search around approximated point
def find_region_to_search(highRes_index):
    global highRes_x, highRes_y
    
    region = np.zeros(121, dtype=np.int32)

    index = 0
    for i in range(-5, 5+1):
        for j in range(-5, 5+1):
            region[index] = np.int32(round(highRes_index + highRes_y * i + j))
            index += 1

    return region

# Find the best suited point from the region on the highRes map
def find_pos_highRes(hist, region):
    global map_pos_high, HR_i
    global func_match_pos

    min_error: float = 100.0
    best_index: int = 0

    for i in range(len(region)):
        if region[i] < 0 or region[i] >= HR_i:
            continue
        error = func_match_pos(hist, map_pos_high[region[i]])
        if min_error > error:
            min_error = error
            best_index = region[i]

    return best_index

# Find the best suited orient for the chosen highRes point
def find_orient_highRes(hist, highRes_index):
    global hist_orient_limits, map_orient_high

    degree_per_limit = 360.0 / hist_orient_limits
    
    best_index = 0
    min_error = float('inf')
    for shift in range(hist_orient_limits):
        error = 0.0
        for j in range(hist_orient_limits):
            pos = j + shift
            pos = pos - hist_orient_limits if pos >= hist_orient_limits else pos
            error += abs(map_orient_high[highRes_index, pos] - hist[j])
        if min_error > error:
            min_error = error
            best_index = shift
    
    orient_aprox = degree_per_limit * best_index
    return orient_aprox
#=======================================================================================#

# APPROXIMATED POINT SEARCHING

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
            map_orient[index, :] = func_compress_orient(raw_data)
            map_loc[index, :] = np.array(json_line["pos"], dtype=np.float32)

    return (map_pos, map_orient, map_loc)

def cooficients_cacl():
    k = LR / HR
    lowRes_x = (x_end - x_begin) / LR + 1
    lowRes_y = (y_end - y_begin) / LR + 1
    highRes_x = (x_end - x_begin) / HR + 1
    highRes_y = (y_end - y_begin) / HR + 1

    return (lowRes_x, lowRes_y, highRes_x, highRes_y, k)

def raw_data_preprocessing(data):
    for i in range(len(data)):
        data[i] = 0.0 if data[i] == float('-inf') else data[i]
        data[i] = float(range_max) if data[i] == float('inf') else data[i]
        data[i] = data[i] - 0.001 if data[i] == range_max else data[i]

    return data
#=======================================================================================#

if __name__ == '__main__':
    # ROS 
    rospy.init_node("map_localization")
    rate = rospy.Rate(1)

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
    # Calculate some needed parameters
    (lowRes_x, lowRes_y, highRes_x, highRes_y, k) = cooficients_cacl()

    # Choose compression and matching functions
        # pos compression function
    if func_compress_pos == "absolute":
        func_compress_pos = compress_pos_absolute
        # orient compress function
    if func_compress_orient == "minInSector":
        func_compress_orient = compress_orient_minInSector
    elif func_compress_orient == "average":
        func_compress_orient = compress_orient_average
        # pos match function
    if func_match_pos == "absolute":
        func_match_pos = match_pos_absolute
    #if func_match_orient == "absolute":
    #    func_match_orient = 

    # Buffer maps
    rospy.loginfo("Buffer maps from files start")
    (map_pos_low, map_orient_low, map_loc_low) = buffer_map(PATH_TO_MAP_LOW_RES, LR_i)
    (map_pos_high, map_orient_high, map_loc_high) = buffer_map(PATH_TO_MAP_HIGH_RES, HR_i)
    rospy.loginfo("Buffer maps from files done")
    rospy.loginfo(map_orient_high)
    rospy.loginfo("================================================================")

    # Main loop
    while not rospy.is_shutdown():
        start_time = time.time()

        # Read laser raw_data and convert them into histogram
        ranges = laser_topic_read()
        hist_pos = func_compress_pos(ranges)
        hist_orient = func_compress_orient(ranges)
        rospy.loginfo("Raw data was read and compressed into hist_posograms")

        # Find best candidate on the low resolution map
        best_index_low = find_pos_lowRes(hist_pos)
        best_low = np.array(map_loc_low[best_index_low])
        rospy.loginfo("Best candidate on the low resolution map:")
        rospy.loginfo("x_low = %.5f, y_high = %.5f", best_low[0], best_low[1])

        # Find nearest points on the high resolution map according to the best low resolution candidate
        highRes_corresponding_index = find_corresponding_point(best_index_low)
        #rospy.loginfo("highRes_corresponding_index = %d", highRes_corresponding_index)
        region_to_search = find_region_to_search(highRes_corresponding_index)
        #rospy.loginfo("Nearest positions to check on the high resolution map")
        #rospy.loginfo("region_to_search")

        best_index_high = find_pos_highRes(hist_pos, region_to_search)
        best_high = np.array(map_loc_high[best_index_high])
        rospy.loginfo("Best candidate on the high resolution map:")
        rospy.loginfo("x_high = %.5f, y_high = %.5f", best_high[0], best_high[1])

        #rospy.loginfo("region:")
        #rospy.loginfo(region_to_search)
        rospy.loginfo("Best index low = %d, best index high = %d", best_index_low, best_index_high)

        orient = find_orient_highRes(hist_orient, best_index_high)
        rospy.loginfo("Approximated orientation = %.5f", orient)


        #rospy.loginfo("Approximated position")
        #rospy.loginfo("x_approx = %.5f, y_approx = %.5f", approx[0], approx[1])


        rospy.loginfo("This step took %.3f", (time.time() - start_time))
        rospy.loginfo("================================================================")
        rate.sleep()
#=======================================================================================#