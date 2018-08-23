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
idno = 0

# find filename given specific pattern
def find(pattern, names):
    result = []
    for name in names:
        if fnmatch.fnmatch(name, pattern):
            result.append(name)
    return result[0]


if __name__ == '__main__':
    rospy.init_node("analyze")
    rospy.loginfo("Analyzing data...")

    path_pub = rospy.Publisher("/path", Path, queue_size=2)
    marker_pub = rospy.Publisher("/marker_array", MarkerArray, queue_size=2)
    global_pub = rospy.Publisher("/global_path", Path, queue_size=2)
    simulations = []

    # load and sort files
    files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    files.sort(key=lambda f: int(filter(str.isdigit, f)))

    # divide into a 2d array based on simulation
    length = len(files)
    divisor = 8
    divisions = length / divisor
    runs_per_planner = 29

    for i in range (0, divisions):
        simulations.append(files[i*divisor : (i+1)*divisor])

    data = [["Run Number", "Timestamp", "Description", "Planner", "dpx", "dpy", "dtheta (radians)", "o-avg", "o-max", "o-min",
            "Run Number", "Timestamp", "Description", "Planner", "dpx", "dpy", "dtheta (radians)", "o-avg", "o-max", "o-min"]]

    c = Convert()

    # extract and analyze data
    for simulation in simulations:
        rospy.loginfo("Analyzing simulation {}...".format(simulations.index(simulation)))
        goal = find("*goal.json", simulation)
        rviz = find("*rviz.json", simulation)
        gazebo = find("*gazebo.json", simulation)
        obstacle = find("*obstacle.json", simulation)
        person = find("*person.json", simulation)
        planner = find("*plan.json", simulation)
        description = find("*description.json", simulation)

        with open(directory + '/simulations/' + goal) as f:
            g = json.load(f)

        with open(directory + '/simulations/' + rviz) as f1:
            r = json.load(f1)

        with open(directory + '/simulations/' + gazebo) as f2:
            t = json.load(f2)

        with open(directory + '/simulations/' + obstacle) as f3:
            o = json.load(f3)

        with open(directory + '/simulations/' + person) as f4:
            p = json.load(f4)

        with open(directory + '/simulations/' + planner) as f5:
            plan = json.load(f5)

        with open(directory + '/simulations/' + description) as f6:
            d = json.load(f6)

        quaternion = (g["goal"][-1]["poses"][0]["orientation"]["x"], g["goal"][-1]["poses"][0]["orientation"]["y"],
                      g["goal"][-1]["poses"][0]["orientation"]["z"], g["goal"][-1]["poses"][0]["orientation"]["w"])

        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        theta = yaw * 180 / math.pi

        if g["planner"] == "robot_only":
            xy = c.robot_to_human(g["goal"][-1]["poses"][0]["position"]["x"], g["goal"][-1]["poses"][0]["position"]["y"], theta)
            dpx = abs(p["person"][-1]["pose"]["position"]["x"] - xy[0])
            dpy = abs(p["person"][-1]["pose"]["position"]["y"] - xy[1])
        else:
            dpx = abs(p["person"][-1]["pose"]["position"]["x"] - g["goal"][-1]["poses"][0]["position"]["x"])
            dpy = abs(p["person"][-1]["pose"]["position"]["y"] - g["goal"][-1]["poses"][0]["position"]["y"])

        p_euler = tf.transformations.euler_from_quaternion((p["person"][-1]["pose"]["orientation"]["x"], p["person"][-1]["pose"]["orientation"]["y"], p["person"][-1]["pose"]["orientation"]["z"],
        p["person"][-1]["pose"]["orientation"]["w"]))

        g_euler = tf.transformations.euler_from_quaternion((g["goal"][-1]["poses"][0]["orientation"]["x"], g["goal"][-1]["poses"][0]["orientation"]["y"], g["goal"][-1]["poses"][0]["orientation"]["z"],
        g["goal"][-1]["poses"][0]["orientation"]["w"]))

        yaw_diff = abs(p_euler[2] - g_euler[2])

        obstacles = o["obstacle"]
        total = 0
        newArray = []
        for obstacle in obstacles:
            total += obstacle["data"]
            newArray.append(obstacle["data"])

        if len(newArray) > 0:
            maximum = max(newArray)
            minimum = min(newArray)
            avg = total / len(obstacles)
        else:
            maximum = minimum = avg = "N/A"

        array = [(simulations.index(simulation) % runs_per_planner) + 1, simulation[0][0:15], d["description"][0]["data"], g["planner"], dpx, dpy, yaw_diff, avg, maximum, minimum]
        if simulations.index(simulation) < runs_per_planner:
            data.append(array)
        else:
            data[(simulations.index(simulation) % runs_per_planner) + 1] += array

    time_string = time.strftime("%Y%m%d-%H%M%S")
    csv_name = time_string + "data.csv"
    with open(directory + '/results/' + csv_name, "w+") as f:
        csvWriter = csv.writer(f, delimiter=',')
        csvWriter.writerows(data)
