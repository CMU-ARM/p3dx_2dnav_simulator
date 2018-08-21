#!/usr/bin/env python

import re
import os
import tf
import time
import json
import math
import rospy
import rospkg
import signal
import subprocess

from convert import Convert
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseArray
from podi_move_base_msgs.msg import PodiMoveBaseActionGoal, PodiMoveBaseActionResult


class Simulation:

    def __init__(self):
        self._child = subprocess.Popen(["roslaunch","p3dx_2dnav","robot.launch"])
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptySrv)
        self._unpause = rospy.ServiceProxy("/gazebo/unpause_physics", EmptySrv)
        self._reset = rospy.ServiceProxy("/gazebo/reset_simulation", EmptySrv)
        self._rviz_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=2, latch=True)
        self._goal_pub = rospy.Publisher("/podi_move_base/goal", PodiMoveBaseActionGoal, queue_size=2)
        self._result_sub = rospy.Subscriber("/podi_move_base/result", PodiMoveBaseActionResult, self._result_cb, queue_size=1)
        self._endgoal_pub = rospy.Publisher("/endgoals", PoseArray, queue_size=2)
        self._description_pub = rospy.Publisher("/description", String, queue_size=1)
        self._restart = False

    def _result_cb(self, msg):
        if msg.status.text == "Goal reached.":
            self._restart = True

    def _simulate(self, planner, start=None, finish=None):
	    # reset simulation if needed
        if self._restart:
            self._restart = False
            if planner == "robot_only":
                self._child = subprocess.Popen(["roslaunch","p3dx_2dnav","robot.launch"])
            elif planner == "coupled":
                self._child = subprocess.Popen(["roslaunch","p3dx_2dnav","coupled.launch"])

            rospy.wait_for_service("/gazebo/pause_physics")
            try:
                self._pause()
            except Exception, e:
                rospy.logerr("Error on calling service: %s", str(e))

            rospy.wait_for_service("/gazebo/reset_simulation")
            try:
                self._reset()
            except Exception, e:
                rospy.logerr("Error on calling service: %s", str(e))

        # pause
        time.sleep(1)
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self._pause()
        except Exception, e:
            rospy.logerr("Error on calling service: %s", str(e))

        # set initial pose
        gazebo_pose = Pose()
        rviz_pose = Pose()
        initial = PoseWithCovarianceStamped()
        c = Convert()
        
        gazebo_start = start["ground_truth"][0]["pose"]["pose"]
        rviz_start = start["robot"][0]["pose"]
        
        gazebo_pose.position.x = gazebo_start["position"]["x"]
        gazebo_pose.position.y = gazebo_start["position"]["y"]
        gazebo_pose.position.z = 0 

        gazebo_pose.orientation.x = gazebo_start["orientation"]["x"]
        gazebo_pose.orientation.y = gazebo_start["orientation"]["y"]
        gazebo_pose.orientation.z = gazebo_start["orientation"]["z"]
        gazebo_pose.orientation.w = gazebo_start["orientation"]["w"]

        rviz_pose.position.x = rviz_start["position"]["x"]
        rviz_pose.position.y = rviz_start["position"]["y"]
        rviz_pose.position.z = 0 

        rviz_pose.orientation.x = rviz_start["orientation"]["x"]
        rviz_pose.orientation.y = rviz_start["orientation"]["y"]
        rviz_pose.orientation.z = rviz_start["orientation"]["z"]
        rviz_pose.orientation.w = rviz_start["orientation"]["w"]
        
        initial.pose.pose = rviz_pose
        initial.header.frame_id = "map"
        initial.pose.covariance = [0.01, -0.00012, 0.0, 0.0, 0.0, 0.0, -0.00012, 0.009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.009]

        state = ModelState()
        state.model_name = "p3dx"
        state.pose = gazebo_pose
        
        # set initial parameters and unpause
        for i in range(0, 2):
            rospy.wait_for_service("/gazebo/set_model_state")
            try:
                ret = self._set_state(state)
                if not ret.success:
                    rospy.loginfo(ret.status_message)
                    self._restart = True
                    self._kill()
                    self._simulate(planner, start, finish)
            except Exception, e:
                rospy.logerr("Error on calling service: %s", str(e))
                self._restart = True
                self._kill()
        
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self._unpause()
        except Exception, e:
            rospy.logerr("Error on calling service: %s", str(e))

        for i in range(0, 100000):
            self._rviz_pub.publish(initial)
        time.sleep(1)

        # set goal
        msg = PodiMoveBaseActionGoal()
        msg.goal.target_poses.header.frame_id = "map"

        end_goals = PoseArray()

        for goal in finish:
            self._description_pub.publish(goal["description"])
            if planner == "robot_only":
                new_xy = c.human_to_robot(goal["x"], goal["y"], goal["angle"])
                new_pose = Pose()
                new_pose.position.x = new_xy[0]
                new_pose.position.y = new_xy[1]
                new_pose.position.z = 0
            elif planner == "coupled":
                new_pose = Pose()
                new_pose.position.x = goal["x"]
                new_pose.position.y = goal["y"]
                new_pose.position.z = 0
            else:
                rospy.logerr("Invalid planner input to simulation function")
                self._kill()

            fangle = goal["angle"] * math.pi / 180
            qfinish = qstart = tf.transformations.quaternion_from_euler(0, 0, fangle)
            new_pose.orientation.x = qfinish[0]
            new_pose.orientation.y = qfinish[1]
            new_pose.orientation.z = qfinish[2]
            new_pose.orientation.w = qfinish[3]

            msg.goal.target_poses.poses.append(new_pose)
            end_goals.header.frame_id = "map"
            end_goals.poses.append(new_pose)

        time.sleep(3)
        for j in range (0, 10):
            self._endgoal_pub.publish(end_goals)
            self._goal_pub.publish(msg)

    def _kill(self):
        self._child.send_signal(signal.SIGINT)
        self._child.wait()

    def _wait(self):
        while not rospy.is_shutdown():
            if self._restart:
                time.sleep(3)
                break

# file sort key
_nsre = re.compile('([0-9]+)')
def natural_sort_key(s):
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split(_nsre, s)] 
            
if __name__ == '__main__':
    rospy.init_node("simulate")

    rospack = rospkg.RosPack()
    directory = rospack.get_path("p3dx_2dnav")
    jsonfile = "goals.json"
    filepath = directory + "/json/" + jsonfile

    with open(filepath) as f:
        data = json.load(f)

    s = Simulation()
    
    path = directory + "/json"
    files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
    print files

    files.sort(key=natural_sort_key)
    files.remove("goals.json")
    
    for startfile in files:
        
        index = files.index(startfile)
        with open(directory + '/json/' + startfile) as f:
            start_data = json.load(f)
            
        if index == 20:
            s._simulate("robot_only", start_data, data["trajectory"]["6"])
            s._wait()
            s._kill()
        rospy.loginfo("Done with Robot-Only Simulation: {}, Simulations to go: {}".format(index + 1, 57 - index))
        
    for startfile in files:
        
        index = files.index(startfile)
        with open(directory + '/json/' + startfile) as f:
            start_data = json.load(f)
            
        #if index == 5:
        #    s._simulate("coupled", start_data, data["trajectory"]["1"]) 
        #    s._wait()
        #    s._kill()
        #rospy.loginfo("Done with Coupled Simulation {}, Simulations to go: {}".format(index + 1, 28 - index))
        
    #analyze1 = subprocess.Popen(["rosrun","p3dx_2dnav","analyze_data.py"])
    #analyze2 = subprocess.Popen(["rosrun","p3dx_2dnav","cycles.py"])
