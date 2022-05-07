import rospy
import json
from math import floor
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState

from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['scripts'],
    package_dir={'': ''}
)
setup(**d)

# Read from laser topic
def laser_topic_read():
    laser_topic_name = "/laser/scan"
    laser = rospy.wait_for_message(laser_topic_name, LaserScan)
    distances = laser.ranges
    range_min = laser.range_min
    range_max = laser.range_max
    return (distances, range_min, range_max)

# Compress distances from laser topic into smaller histogram
def compress_laser_data(data: list, range_min: float, range_max: float, limits: int):
    length = len(data)
    ran = range_max / limits
    out = [0 for i in range(limits)]

    for item in data:
        lim = floor(item / ran)
        out[lim] = out[lim] + 1

    return out

# Move a robot on the designated position in the Gazebo simulation
# phi is always == 0 (for now)
def robot_move_on(publisher, robot: ModelState, x: float, y: float):
    # Model name and world
    robot_state.model_name = "p3dx"
    robot_state.reference_frame = 'willowgarage'

    # Position
    robot_state.pose.position.x = x
    robot_state.pose.position.y = y
    robot_state.pose.position.z = 0.05

    # Quaterion orientation
    robot_state.pose.orientation.w = 1
    robot_state.pose.orientation.x = 0
    robot_state.pose.orientation.y = 0
    robot_state.pose.orientation.z = 0

    # Publish
    pub.publish(robot_state)

# Append one line to file
def append_to_file(file_path, pos, distances, compressed):
    with open(PATH_TO_FOLDER, 'a') as jsonfile:
        json.dump({"pos": pos, "dist": distances, "hist": compressed}, jsonfile)
        jsonfile.write(str('\n'))