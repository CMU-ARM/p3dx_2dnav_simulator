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
from std_msgs.msg import Float64, String
from rosgraph_msgs.msg import Log

rospack = rospkg.RosPack()
directory = rospack.get_path("p3dx_2dnav")
timestr = time.strftime("%Y%m%d-%H%M%S")
rname1 = timestr + "rviz.json"
rname2 = timestr + "gazebo.json"
pname = timestr + "person.json"
oname = timestr + "obstacle.json"
gname = timestr + "goal.json"
dname = timestr + "description.json"
rosout_name = timestr + "rosout.json"
planner_name = timestr + "plan.json"
rpath1 = directory + '/simulations/' + rname1
rpath2 = directory + '/simulations/' + rname2
ppath = directory + '/simulations/' + pname
opath = directory + '/simulations/' + oname
gpath = directory + '/simulations/' + gname
dpath = directory + '/simulations/' + dname
planner_path = directory + '/simulations/' + planner_name
rosout_path = directory + '/simulations/' + rosout_name
rviz = {"rviz": []}
gazebo = {"gazebo": []}
obstacle = {"obstacle": []}
goal = {"goal": [], "planner": ""}
person = {"person": []}
planner = {"planner": []}
description = {"description": []}
rosout = {"rosout": []}

class Record(object):
    def __init__(self):
        # robot and human poses
        self._rviz_sub = rospy.Subscriber('/convert_poses/pose_converted', PoseStamped, self._rviz_cb, queue_size=1)
        self._gazebo_sub = rospy.Subscriber('/p3dx/base_pose_ground_truth', Odometry, self._gazebo_cb, queue_size=1)
        self._person_sub = rospy.Subscriber('/coupling_model_node/human_position', PoseStamped, self._person_cb, queue_size=1)
        # pose msg params
        self._last_rviz = None
        self._last_gazebo = None
        self._last_person = None
        
        # obstacle distance from human and obstalce msg param
        self._obstacle_sub = rospy.Subscriber('/obstacle_dist', Float64, self._obstacle_cb, queue_size=1)
        self._last_obstacle = None
        
        # global path and path msg param
        self._planner_sub = rospy.Subscriber('/podi_move_base/GlobalPlanner/robot_global_plan', Path, self._planner_cb, queue_size=1)
        self._last_path = None
        
        # goal and goal msg params
        self._r_endgoal_sub = rospy.Subscriber("/podi_move_base/GlobalPlanner/robot_traj_goal", PoseArray, self._rend_cb, queue_size=1)
        self._c_endgoal_sub = rospy.Subscriber("/podi_move_base/GlobalPlanner/human_traj_goal", PoseArray, self._cend_cb, queue_size=1)
        self._last_rgoal = None
        self._last_cgoal = None
        
        # description and msg
        self._description_sub = rospy.Subscriber("/description", String, self._description_cb, queue_size=1)
        self._last_description = None
        
        # rosout and msg
        self._rosout_sub = rospy.Subscriber('/rosout', Log, self._rosout_cb, queue_size=200)
        self._last_rosout = None

    def _rviz_cb(self, msg):
        if msg != self._last_rviz:
            y = yaml.load(str(msg))
            rviz["rviz"].append(y)
        self._last_rviz = msg  
        
    def _gazebo_cb(self, msg):
        if msg != self._last_gazebo:
            y = yaml.load(str(msg))
            gazebo["gazebo"].append(y)
        self._last_gazebo = msg          

    def _person_cb(self, msg):
        if self._last_person != None and (msg.pose.position != self._last_person.pose.position or msg.pose.orientation != self._last_person.pose.orientation):
            y = yaml.load(str(msg))
            person["person"].append(y)
        self._last_person = msg
	
    def _obstacle_cb(self, msg):
        if msg != self._last_obstacle and self._last_obstacle != None:
            y = yaml.load(str(msg))
            obstacle["obstacle"].append(y)
        self._last_obstacle = msg
	    
    def _planner_cb(self, msg):
        if msg != self._last_path:
            y = yaml.load(str(msg))
            planner["planner"].append(y)
        self._last_plan = msg
                
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
    
    def _description_cb(self, msg):
        if self._last_description == None:
            y = yaml.load(str(msg))
            description["description"].append(y)
        self._last_description = msg
    
    def _rosout_cb(self, msg):
        if self._last_rosout != None:
            if msg.msg and msg.msg[0:4] == "Best":
                data = msg.msg.split(' ')
                range_value = data[7][:-1]
                cycles_value = data[-1]
                dictionary = {"range": "", "cycles": ""}
                dictionary["range"] = range_value
                dictionary["cycles"] = cycles_value
                rosout["rosout"].append(yaml.load(str(dictionary)))
        self._last_rosout = msg

def handle():
    if len(rviz["rviz"]) > 1:
        with open(rpath1, "w+") as f:
            f.write(json.dumps(rviz, indent=4))
    
        with open(rpath2, "w+") as f:
            f.write(json.dumps(gazebo, indent=4))
    
        with open(ppath, "w+") as f:
            f.write(json.dumps(person, indent=4))

        with open(opath, "w+") as f:
            f.write(json.dumps(obstacle, indent=4))

        with open(gpath, "w+") as f:
            f.write(json.dumps(goal, indent=4))

        with open(planner_path, "w+") as f:
            f.write(json.dumps(planner, indent=4))
            
        with open(dpath, "w+") as f:
            f.write(json.dumps(description, indent=4))
    
        with open(rosout_path, "w+") as f:
            f.write(json.dumps(rosout, indent=4))
    
if __name__ == '__main__':
    rospy.init_node("data")
    r = Record()
    rospy.on_shutdown(handle)
    rospy.spin()
