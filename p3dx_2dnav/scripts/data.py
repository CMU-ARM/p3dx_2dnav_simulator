#!/usr/bin/env python

import os
import rospy
import yaml
import json
import rospkg
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, PoseArray
from podi_move_base_msgs.msg import PodiMoveBaseActionGoal
from nav_msgs.msg import Path
from std_msgs.msg import Float64

rospack = rospkg.RosPack()
directory = rospack.get_path("p3dx_2dnav")
timestr = time.strftime("%Y%m%d-%H%M%S")
rname = timestr + "podi.json"
pname = timestr + "person.json"
oname = timestr + "obstacle.json"
gname = timestr + "goal.json"
planner_name = timestr + "plan.json"
rpath = directory + '/simulations/' + rname
ppath = directory + '/simulations/' + pname
opath = directory + '/simulations/' + oname
gpath = directory + '/simulations/' + gname
planner_path = directory + '/simulations/' + planner_name
podi = {"podi": []}
obstacle = {"obstacle": []}
goal = {"goal": [], "planner": ""}
person = {"person": []}
planner = {"planner": []}

class Record(object):
    def __init__(self):
        self._rpose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._rpose_cb, queue_size=1)
        self._ppose_sub = rospy.Subscriber('/coupling_model_node/human_position', PoseStamped, self._ppose_cb, queue_size=1)
        self._obstacle_sub = rospy.Subscriber('/obstacle_dist', Float64, self._obstacle_cb, queue_size=1)
        #self._goal_sub = rospy.Subscriber('/podi_move_base/goal', PodiMoveBaseActionGoal, self._goal_cb, queue_size=1)
        #self._goal_sub = rospy.Subscriber('/podi_move_base/current_goals', PoseArray, self._goal_cb, queue_size=1)
        self._planner_sub = rospy.Subscriber('/podi_move_base/GlobalPlanner/robot_global_plan', Path, self._planner_cb, queue_size=1)
        self._r_endgoal_sub = rospy.Subscriber("/podi_move_base/GlobalPlanner/robot_traj_goal", PoseArray, self._rend_cb, queue_size=1)
        self._c_endgoal_sub = rospy.Subscriber("/podi_move_base/GlobalPlanner/human_traj_goal", PoseArray, self._cend_cb, queue_size=1)
        self._pose_pub = rospy.Publisher('/robot_position', PoseStamped, queue_size=2)
        self._rlast_pose = None
        self._plast_pose = None
        self._last_obstacle = None
        #self._last_goal = None
        self._last_rgoal = None
        self._last_cgoal = None
        self._last_path = None

    def _rpose_cb(self, msg):
    	if msg != None:
	        new_msg = PoseStamped()
	        new_msg.header = msg.header
	        new_msg.pose.position = msg.pose.pose.position
	        new_msg.pose.orientation = msg.pose.pose.orientation
	        self._pose_pub.publish(new_msg)

        if msg != self._rlast_pose:
            y = yaml.load(str(msg))
            podi["podi"].append(y)
        self._rlast_pose = msg      

    def _ppose_cb(self, msg):
        if self._plast_pose != None and (msg.pose.position != self._plast_pose.pose.position or msg.pose.orientation != self._plast_pose.pose.orientation):
            y = yaml.load(str(msg))
            person["person"].append(y)
        self._plast_pose = msg
	
    def _obstacle_cb(self, msg):
        if msg != self._last_obstacle and self._last_obstacle != None:
            y = yaml.load(str(msg))
            obstacle["obstacle"].append(y)
        self._last_obstacle = msg

    '''def _goal_cb(self, msg):
        if self._last_goal == None:
            y = yaml.load(str(msg))
            x = rospy.get_param("/podi_move_base/base_global_planner")
            if x == "podi_robot_only_planner/GlobalPlanner":
	            goal["planner"] = "robot_only"
            else:
                goal["planner"] = "coupled"
                goal["goal"].append(y)
	    self._last_goal = msg'''
    def _goal_cb(self, msg):
        if self._last_goal != None:
            y = yaml.load(str(msg))
            x = rospy.get_param("/podi_move_base/base_global_planner")
            if x == "podi_robot_only_planner/GlobalPlanner":
                goal["planner"] = "robot_only"
            else:
                goal["planner"] = "coupled"
            goal["goal"].append(y)
        self._last_goal = msg
                
    def _rend_cb(self, msg):
        if self._last_rgoal != None:
            y = yaml.load(str(msg))
            x = rospy.get_param("/podi_move_base/base_global_planner")
            if x == "podi_robot_only_planner/GlobalPlanner":
                goal["planner"] = "robot_only"
                goal["goal"].append(y)
        self._last_rgoal = msg

    def _cend_cb(self, msg):
        if self._last_cgoal != None:
            y = yaml.load(str(msg))
            goal["planner"] = "coupled"
            goal["goal"].append(y)
        self._last_cgoal = msg

    def _planner_cb(self, msg):
        if msg != self._last_path:
            y = yaml.load(str(msg))
            planner["planner"].append(y)
        self._last_plan = msg
	

def handle():
    if len(podi["podi"]) > 1:
        with open(rpath, "w+") as f:
            f.write(json.dumps(podi, indent=4))
    
    if len(person["person"]) > 1:
        with open(ppath, "w+") as f:
            f.write(json.dumps(person, indent=4))

    if len(obstacle["obstacle"]) > 1:
        with open(opath, "w+") as f:
            f.write(json.dumps(obstacle, indent=4))

    if len(podi["podi"]) > 1:
        with open(gpath, "w+") as f:
            f.write(json.dumps(goal, indent=4))

    if len(podi["podi"]) > 1:
        with open(planner_path, "w+") as f:
            f.write(json.dumps(planner, indent=4))

    
if __name__ == '__main__':
    rospy.init_node("data")
    r = Record()
    rospy.on_shutdown(handle)
    rospy.spin()
