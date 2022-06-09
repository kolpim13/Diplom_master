#!/usr/bin/env python
import rospy
import sys
import pathlib
import json
import math

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan
#=======================================================================================#

def laser_topic_read():
    laser_topic_name = "/laser/scan"
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    distances = [round(item, 5) for item in laser.ranges]
    return distances

def model_move_on(x: float, y: float):
    global pub_model_state, model_state

    # Position
    model_state.pose.position.x = x
    model_state.pose.position.y = y

    # Publish
    pub_model_state.publish(model_state)

def append_to_file(file_path, pos, distances):
    with open(file_path, 'a') as jsonfile:
        json.dump({"pos": pos, "raw_data": distances}, jsonfile)
        jsonfile.write(str('\n'))

def get_real_robot_pos():
    gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    robot_pose = gms(gazebo_model_name, "")
    real_x = robot_pose.pose.position.x
    real_y = robot_pose.pose.position.y
    return (real_x, real_y)

def generate_descriptor(path_to_desc):
    global pos_x_begin, pos_y_begin, pos_x_end, pos_y_end, lowRes_step, highRes_step 
    global lowRes_lines, highRes_lines

    obj = {
        "x_begin": str(pos_x_begin),
        "y_begin": str(pos_y_begin),
        "x_end": str(pos_x_end),
        "y_end": str(pos_y_end),
        "lowRes": str(lowRes_step),
        "highRes": str(highRes_step),
        "lowRes_i": str(lowRes_lines),
        "highRes_i": str(highRes_lines),
    }

    with open(path_to_desc, 'w+') as jsonfile:
        json.dump(obj, jsonfile)

def generate_map(path_to_map, step) -> int:
    global pos_x_begin, pos_y_begin, pos_x_end, pos_y_end, lowRes_step, highRes_step, laser_range_max

    with open(path_to_map, 'w+') as jsonfile:
        pass

    x = pos_x_begin
    y = pos_y_begin
    i: int = 0

    while y <= pos_y_end:
        while x <= pos_x_end:
            model_move_on(x, y)
            rate.sleep()

            (real_x, real_y) = get_real_robot_pos()
            distances = laser_topic_read()
            for item in distances:
                item = 0.0 if item == float('-inf') else item
                item = laser_range_max if item == float('inf') else item
                item = item - 0.001 if item == laser_range_max else item
            append_to_file(path_to_map, (round(real_x, 1), round(real_y, 1), 0.0), distances)

            i += 1
            if i % 10 == 0:
                rospy.loginfo("x = %f, y = %f done", x, y)
            x += step

        x = pos_x_begin
        y += step
    return i

def generate_map_void():
    global PATH_TO_MAP_LOW_RES

    with open(PATH_TO_MAP_LOW_RES, 'w+') as jsonfile:
        for i in range(10):
            model_move_on(0, 0)
            rate.sleep()
            (real_x, real_y) = get_real_robot_pos()
#=======================================================================================#

if __name__ == '__main__':
    # ROS 
    rospy.init_node("map_creater")
    rate = rospy.Rate(10)

    # Parameters read
    map_name = rospy.get_param("~map_name")
    gazebo_model_name = rospy.get_param("~gazebo_model_name")
    reference_frame = rospy.get_param("~reference_frame")
    pos_x_begin = rospy.get_param("~pos_x_begin")
    pos_y_begin = rospy.get_param("~pos_y_begin")
    pos_x_end = rospy.get_param("~pos_x_end")
    pos_y_end = rospy.get_param("~pos_y_end")
    lowRes_step = rospy.get_param("~lowRes_step")
    highRes_step = rospy.get_param("~highRes_step")
    laser_range_min = rospy.get_param("~laser_min")
    laser_range_max = rospy.get_param("~laser_max")

    BASE_DIR = pathlib.Path(__file__).resolve(strict=True).parent
    PATH_TO_MAP_FOLDER = pathlib.Path(BASE_DIR.parent, "records")
    PATH_TO_MAP_DESCRIPTOR = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_description.txt")
    PATH_TO_MAP_LOW_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_lowRes.txt")
    PATH_TO_MAP_HIGH_RES = pathlib.Path(PATH_TO_MAP_FOLDER, map_name + "_highRes.txt")

    # Publishers and subscribers
    model_state = ModelState()
    pub_model_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)

    # Adjusting
    model_state.model_name = gazebo_model_name
    model_state.reference_frame = reference_frame
    model_state.pose.position.z = 0.1
    model_state.pose.orientation.w = 1
    model_state.pose.orientation.x = 0
    model_state.pose.orientation.y = 0
    model_state.pose.orientation.z = 0

    # Generate two maps
    generate_map_void()
    lowRes_lines = generate_map(PATH_TO_MAP_LOW_RES, lowRes_step)
    highRes_lines = generate_map(PATH_TO_MAP_HIGH_RES, highRes_step)

    # Create desriptor for maps
    generate_descriptor(PATH_TO_MAP_DESCRIPTOR)
#=======================================================================================#