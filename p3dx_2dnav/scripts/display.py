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
import signal
import subprocess
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
    rospy.init_node("display")
    rospy.loginfo("Analyzing data...")

    rviz_display = subprocess.Popen(["roslaunch","p3dx_2dnav","robot.launch"])

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

    # display rviz data
    time.sleep(15)
    run = raw_input("What timestamp would you like to see? ")
    for simulation in simulations:
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

        # display based on user input
        if run == simulation[0][0:15] or run == "all" or run == "All":
            rospy.loginfo("Analyzing simulation with timestamp {}...".format(simulation[0][0:15]))
            path = Path()
            marker_array = MarkerArray()

            for timestep in p["person"]:
                marker = Marker()
                marker.type = 0
                marker.id = idno
                idno += 1
                marker.action = 0

                pose = Pose()
                marker.header.frame_id = path.header.frame_id = '/map'
                path_pose = PoseStamped()
                path_pose.header.seq = timestep["header"]["seq"]
                path_pose.header.stamp = rospy.get_rostime()
                path_pose.header.frame_id = '/map'

                pose.position.x = path_pose.pose.position.x = timestep["pose"]["position"]["x"]
                pose.position.y = path_pose.pose.position.y = timestep["pose"]["position"]["y"]
                pose.position.z = path_pose.pose.position.z = timestep["pose"]["position"]["z"]
                pose.orientation.x = path_pose.pose.orientation.x = timestep["pose"]["orientation"]["x"]
                pose.orientation.y = path_pose.pose.orientation.y = timestep["pose"]["orientation"]["y"]
                pose.orientation.z = path_pose.pose.orientation.z = timestep["pose"]["orientation"]["z"]
                pose.orientation.w = path_pose.pose.orientation.w = timestep["pose"]["orientation"]["w"]

                marker.pose = pose
                marker.scale.x = 0.07
                marker.scale.y = 0.02
                marker.scale.z = 0.035
                marker.color.a = 1.0

                if g["planner"] == "coupled":
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0

                path.poses.append(path_pose)
                marker_array.markers.append(marker)

            g_marker = Marker()
            g_marker.header.frame_id = "/map"
            g_marker.type = 0
            g_marker.id = idno
            idno += 1
            g_marker.action = 0

            g2_marker = Marker()
            g2_marker.header.frame_id = "/map"
            g2_marker.type = 0
            g2_marker.id = idno
            idno += 1
            g2_marker.action = 0

            g_marker.pose.position.x = g["goal"][-1]["poses"][0]["position"]["x"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["position"]["x"]
            g_marker.pose.position.y = g["goal"][-1]["poses"][0]["position"]["y"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["position"]["y"]
            g_marker.pose.position.z = g["goal"][-1]["poses"][0]["position"]["z"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["position"]["z"]
            g_marker.pose.orientation.x = g["goal"][-1]["poses"][0]["orientation"]["x"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["orientation"]["x"]
            g_marker.pose.orientation.y = g["goal"][-1]["poses"][0]["orientation"]["y"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["orientation"]["y"]
            g_marker.pose.orientation.z = g["goal"][-1]["poses"][0]["orientation"]["z"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["orientation"]["z"]
            g_marker.pose.orientation.w = g["goal"][-1]["poses"][0]["orientation"]["w"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["orientation"]["w"]
            g_marker.scale.x = 0.07
            g_marker.scale.y = 0.02
            g_marker.scale.z = 0.035
            g_marker.color.a = 1.0
            g_marker.color.r = 1.0
            g_marker.color.g = 0.0
            g_marker.color.b = 0.0


            quaternion = (g["goal"][-1]["poses"][0]["orientation"]["x"], g["goal"][-1]["poses"][0]["orientation"]["y"],
                          g["goal"][-1]["poses"][0]["orientation"]["z"], g["goal"][-1]["poses"][0]["orientation"]["w"])

            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            theta = yaw * 180 / math.pi

            if g["planner"] == "robot_only":
                new_xy = c.robot_to_human(g["goal"][-1]["poses"][0]["position"]["x"], g["goal"][-1]["poses"][0]["position"]["y"], theta)
            else:
                new_xy = c.human_to_robot(g["goal"][-1]["poses"][0]["position"]["x"], g["goal"][-1]["poses"][0]["position"]["y"], theta)

            g2_marker.pose.position.x = new_xy[0]
            g2_marker.pose.position.y = new_xy[1]
            g2_marker.pose.position.z = g["goal"][-1]["poses"][0]["position"]["z"] #g["goal"][0]["goal"]["target_poses"]["poses"][0]["position"]["z"]
            g2_marker.pose.orientation.x = g["goal"][-1]["poses"][0]["orientation"]["x"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["orientation"]["x"]
            g2_marker.pose.orientation.y = g["goal"][-1]["poses"][0]["orientation"]["y"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["orientation"]["y"]
            g2_marker.pose.orientation.z = g["goal"][-1]["poses"][0]["orientation"]["z"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["orientation"]["z"]
            g2_marker.pose.orientation.w = g["goal"][-1]["poses"][0]["orientation"]["w"]#g["goal"][0]["goal"]["target_poses"]["poses"][0]["orientation"]["w"]
            g2_marker.scale.x = 0.07
            g2_marker.scale.y = 0.02
            g2_marker.scale.z = 0.035
            g2_marker.color.a = 1.0
            g2_marker.color.r = 1.0
            g2_marker.color.g = 0.0
            g2_marker.color.b = 0.0

            marker_array.markers.append(g_marker)
            marker_array.markers.append(g2_marker)

            global_path = Path()
            global_path.header.frame_id = '/map'

            for pose in plan["planner"][-1]["poses"]:
                gpose = PoseStamped()
                gpose.header.seq = timestep["header"]["seq"]
                gpose.header.stamp = rospy.get_rostime()
                gpose.header.frame_id = '/map'

                gpose.pose.position.x = pose["pose"]["position"]["x"]
                gpose.pose.position.y = pose["pose"]["position"]["y"]
                gpose.pose.position.z = pose["pose"]["position"]["z"]
                gpose.pose.orientation.x = pose["pose"]["orientation"]["x"]
                gpose.pose.orientation.y = pose["pose"]["orientation"]["y"]
                gpose.pose.orientation.z = pose["pose"]["orientation"]["z"]
                gpose.pose.orientation.w = pose["pose"]["orientation"]["w"]

                global_path.poses.append(gpose)

                for i in range (0, 4):
                    path_pub.publish(path)
                    global_pub.publish(global_path)
                    marker_pub.publish(marker_array)
                    time.sleep(0.1)

    rviz_display.send_signal(signal.SIGINT)
    rviz_display.wait()
