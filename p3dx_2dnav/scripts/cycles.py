#!/usr/bin/env python

import os
import csv
import fnmatch
import rospkg
import json
import rospy
import math
import tf
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
from convert import Convert

rospack = rospkg.RosPack()
directory = rospack.get_path("p3dx_2dnav")
path = directory + '/simulations'

# find filename given specific pattern
def find(pattern, names):
    result = []
    for name in names:
        if fnmatch.fnmatch(name, pattern):
            result.append(name)
    return result[0]


if __name__ == '__main__':
    rospy.init_node("analyze_cycles")

    rosout = []
    runs_per_planner = 29

    # load and sort files
    files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    files.sort(key=lambda f: int(filter(str.isdigit, f)))

    # divide into a 2d array based on simulation
    length = len(files)
    divisor = 8
    divisions = length / divisor

    for i in range (0, divisions):
        rosout.append(files[i*divisor : (i+1)*divisor])

    csv_data = ["Run Number", "Max Common Range", "Robot-Only Cycles", "Coupled Cycles"]

    # extract and analyze data
    for simulation in rosout[0:runs_per_planner]:
        rospy.loginfo("Analyzing range and cycles for run {}...".format(rosout.index(simulation)))
        robot_data = find("*rosout.json", simulation)

        index = rosout.index(simulation)
        coupled_data = find("*rosout.json", rosout[index+29])

        with open(directory + '/simulations/' + robot_data) as f:
            r = json.load(f)

        with open(directory + '/simulations/' + coupled_data) as f1:
            c = json.load(f1)

        rrange = []
        rcycle = []

        crange = []
        ccycle = []

        for timestep in r["rosout"]:
            rrange.append(timestep["range"])
            rcycle.append(timestep["cycles"])

        for timestep in c["rosout"]:
            crange.append(timestep["range"])
            ccycle.append(timestep["cycles"])

        max_common_range = max(list(set(rrange).intersection(crange)))
        min_rcycle = next(x for x in rcycle if rrange[rcycle.index(x)] == max_common_range)
        min_ccycle = next(x for x in ccycle if crange[ccycle.index(x)] == max_common_range)
        csv_data.append([index + 1, max_common_range, min_rcycle, min_ccycle])

    formatted_time = time.strftime("%Y%m%d-%H%M%S")
    csv_name = formatted_time + "cycles.csv"
    with open(directory + '/results/' + csv_name) as f:
        csvWriter = csv.writer(f, delimiter=',')
        csvWriter.writerows(csv_data)
