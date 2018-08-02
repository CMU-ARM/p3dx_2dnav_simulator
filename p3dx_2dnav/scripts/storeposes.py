#!/usr/bin/env python

import os
import rospy
import yaml
import json
import rospkg
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, PoseArray
from podi_move_base_msgs.msg import PodiMoveBaseActionGoal
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64

rospack = rospkg.RosPack()
directory = rospack.get_path("p3dx_2dnav")
timestr = time.strftime("%Y%m%d-%H%M%S")
name = raw_input("Number of recorded start: ") + "pose.json"
path = directory + '/json/' + name
poses = {"robot": [], "ground_truth": []}

class Record(object):
    def __init__(self):
        self._pose_sub = rospy.Subscriber('/convert_poses/pose_converted', PoseStamped, self._pose_cb, queue_size=1)
        self._truth_sub = rospy.Subscriber('/p3dx/base_pose_ground_truth', Odometry, self._truth_cb, queue_size=1)
        self._last_pose = None
        self._last_truth = None  

    def _pose_cb(self, msg):
        if self._last_pose == None:
            y = yaml.load(str(msg))
            poses["robot"].append(y)
        self._last_pose = msg
	
    def _truth_cb(self, msg):
        if self._last_truth == None:
            y = yaml.load(str(msg))
            poses["ground_truth"].append(y)
        self._last_truth = msg
    
if __name__ == '__main__':
    rospy.init_node("store_poses")
    r = Record()
    while not rospy.is_shutdown():
        #print(len(poses["robot"]), len(poses["ground_truth"]))
        if len(poses["robot"]) > 0 and len(poses["ground_truth"]) > 0:
            with open(path, "w+") as f:
                f.write(json.dumps(poses, indent=4))
            break
