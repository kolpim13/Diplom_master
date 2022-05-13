#!/usr/bin/env python
import rospy
import sys
import json
import traceback
import math
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan

# Variables
pub = None
robot_state = None
PATH_TO_FILE = "/home/maks/catkin_ws/src/diplom/records/map2.json"
model_to_move = None
laser_topic_to_read = None

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

def robot_move_on(x: float, y: float):
    global pub, robot_state

    # Position
    robot_state.pose.position.x = x
    robot_state.pose.position.y = y

    # Publish
    pub.publish(robot_state)

def append_to_file(file_path, pos, distances, compressed):
    with open(PATH_TO_FILE, 'a') as jsonfile:
        json.dump({"pos": pos, "hist": compressed}, jsonfile)
        jsonfile.write(str('\n'))

def get_real_robot_pos():
    robot_pose_topic_name = ""
    gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    robot_pose = gms("lidar", "")
    real_x = robot_pose.pose.position.x
    real_y = robot_pose.pose.position.y
    return (real_x, real_y)

if __name__ == '__main__':
    rospy.init_node('map_creater')

    #model_to_move = rospy.get_param("model_to_move", "liadr")
    #laser_topic_to_read = rospy.get_param("laser_topic_to_read", "/laser/scan")

    rate = rospy.Rate(0.01)

    robot_state = ModelState()
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
    
    # Robot state adjust
    robot_state.model_name = 'lidar'
    robot_state.reference_frame = 'willowgarage'
    #robot_state.reference_frame = 'cafe'
    robot_state.pose.position.z = 0.05
    robot_state.pose.orientation.w = 1
    robot_state.pose.orientation.x = 0
    robot_state.pose.orientation.y = 0
    robot_state.pose.orientation.z = 0

    # First and last map positions
    x_i = 0.0
    y_i = 0.0
    x_l = 55.1
    y_l = 45.1
    x_step = 0.1    # in meters
    y_step = 0.1    # in meters

    # Create or clear file were we are writing
    with open(PATH_TO_FILE, 'w+') as jsonfile:
        pass

    # Main algorithm
    x = x_i
    y = y_i
    i = 0
    while y <= y_l:
        while x <= x_l:
            robot_move_on(x, y)
            rospy.sleep(0.1)
            (distances, range_min, range_max) = laser_topic_read()
            compressed = compress_laser_data(distances, range_min, range_max, 60)
            (real_x, real_y) = get_real_robot_pos()
            append_to_file(PATH_TO_FILE, (round(real_x, 3), round(real_y, 3), 0.0), distances, compressed)
            i += 1
            if i == 10:
                rospy.loginfo("x = %f, y = %f done", x, y)
                i = 0
            x = x + x_step
        x = x_i
        y = y + y_step

        rate.sleep()


    #while not rospy.is_shutdown():
    #    x += 0.1
    #    y += 0.1
    #    # Position
    #    robot_state.pose.position.x = x
    #    robot_state.pose.position.y = y