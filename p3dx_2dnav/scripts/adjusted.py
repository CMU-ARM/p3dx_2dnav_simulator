#!/usr/bin/env python

import rospy
import rospkg
import json
import math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseArray
from std_srvs.srv import Empty as EmptySrv
from podi_move_base_msgs.msg import PodiMoveBaseActionGoal, PodiMoveBaseActionResult
import subprocess
import signal
import time
import tf
from convert import Convert

class Start:
    def __init__(self):
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptySrv)
        self._unpause = rospy.ServiceProxy("/gazebo/unpause_physics", EmptySrv)
        self._reset = rospy.ServiceProxy("/gazebo/reset_simulation", EmptySrv)
        self._rviz_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=2, latch=True)
        self._goal_pub = rospy.Publisher("/podi_move_base/goal", PodiMoveBaseActionGoal, queue_size=2)
        self._result_sub = rospy.Subscriber("/podi_move_base/result", PodiMoveBaseActionResult, self._result_cb, queue_size=1)
        self._endgoal_pub = rospy.Publisher("/endgoals", PoseArray, queue_size=2)


    def _start(self, robot, truth):
        # pause
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self._pause()
        except Exception, e:
            rospy.logerr("Error on calling service: %s", str(e))

        # set initial pose
        pose = Pose()
        initial = PoseWithCovarianceStamped()
        c = Convert()

        pose.orientation.x = robot["orientation"]["x"]
        pose.orientation.y = robot["orientation"]["y"]
        pose.orientation.z = robot["orientation"]["z"]
        pose.orientation.w = robot["orientation"]["w"]
        
        euler1 = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
        xy = c.human_to_robot(robot["position"]["x"], robot["position"]["y"], euler1[2] * 180 / math.pi)
        pose.position.x = xy[0]
        pose.position.y = xy[1]
        pose.position.z = 0

        initial.pose.pose = pose
        initial.header.frame_id = "map"
        initial.pose.covariance = [0.01, -0.00012, 0.0, 0.0, 0.0, 0.0, -0.00012, 0.009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.009]

        state = ModelState()
        state.model_name = "p3dx"
        state.reference_frame = "map"
        
        gazebo_pose = Pose()
        gazebo_pose.orientation.x = truth["orientation"]["x"]
        gazebo_pose.orientation.y = truth["orientation"]["y"]
        gazebo_pose.orientation.z = truth["orientation"]["z"]
        gazebo_pose.orientation.w = truth["orientation"]["w"]
        
        euler2 = tf.transformations.euler_from_quaternion((gazebo_pose.orientation.x, gazebo_pose.orientation.y, gazebo_pose.orientation.z, gazebo_pose.orientation.w))
        xy1 = c.human_to_robot(truth["position"]["x"], truth["position"]["y"], euler2[2] * 180 / math.pi)
        gazebo_pose.position.x = xy1[0]
        gazebo_pose.position.y = xy1[1]
        gazebo_pose.position.z = 0

        state.pose = gazebo_pose
        
        rospy.loginfo(state)
        rospy.loginfo(initial)
        
        # set initial parameters and unpause
        for i in range(0, 2):
            rospy.wait_for_service("/gazebo/set_model_state")
            try:
                ret = self._set_state(state)
                if not ret.success:
                    rospy.loginfo(ret.status_message)
            except Exception, e:
                rospy.logerr("Error on calling service: %s", str(e))
        
        for i in range(0, 10000):
            self._rviz_pub.publish(initial)
            
        time.sleep(1)
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self._unpause()
        except Exception, e:
            rospy.logerr("Error on calling service: %s", str(e))


        for i in range(0, 10000):
            self._rviz_pub.publish(initial)
        time.sleep(1)


if __name__ == '__main__':
    rospy.init_node("start")

    rospack = rospkg.RosPack()
    directory = rospack.get_path("p3dx_2dnav")
    jsonfile = "20180727-153334pose.json"
    filepath = directory + "/json/" + jsonfile

    with open(filepath) as f:
        data = json.load(f)

    s = Start()

    # example usage
    s._start(data["robot"][0]["pose"], data["ground_truth"][0]["pose"]["pose"])
