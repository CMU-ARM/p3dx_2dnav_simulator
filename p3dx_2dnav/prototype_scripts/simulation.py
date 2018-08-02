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

class Simulation:
    def __init__(self):
        self._child = subprocess.Popen(["roslaunch","p3dx_2dnav","p3dx_2dnav.launch"])
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptySrv)
        self._unpause = rospy.ServiceProxy("/gazebo/unpause_physics", EmptySrv)
        self._reset = rospy.ServiceProxy("/gazebo/reset_simulation", EmptySrv)
        self._rviz_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=2, latch=True)
        self._goal_pub = rospy.Publisher("/podi_move_base/goal", PodiMoveBaseActionGoal, queue_size=2)
        self._result_sub = rospy.Subscriber("podi_move_base/result", PodiMoveBaseActionResult, self._result_cb, queue_size=1)
        self._restart = False
    
    def _result_cb(self, msg):
        if msg.status.text == "Goal reached.":
            self._restart = True

    def _simulate(self, planner, start=None, finish=None):
	    # reset simulation if needed
        if self._restart:
            self._restart = False
            self._child = subprocess.Popen(["roslaunch","p3dx_2dnav","p3dx_2dnav.launch"])

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
        pose = Pose()
        initial = PoseWithCovarianceStamped()
        c = Convert()

        xy = c.human_to_robot(start["x"], start["y"], start["angle"])
        pose.position.x = xy[0]
        pose.position.y = xy[1]
        pose.position.z = 0

        sangle = start["angle"] * math.pi / 180
        qstart = tf.transformations.quaternion_from_euler(0, 0, sangle)
        pose.orientation.x = qstart[0]
        pose.orientation.y = qstart[1]
        pose.orientation.z = qstart[2]
        pose.orientation.w = qstart[3]

        initial.pose.pose = pose
        initial.header.frame_id = "map"
        initial.pose.covariance = [0.009992497517046672, -0.0001387644792316678, 0.0, 0.0, 0.0, 0.0, -0.00013876447921745694, 0.009840109247164719, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.009882446621228757]

        state = ModelState()
        state.model_name = "p3dx"
        state.pose = pose

        # set initial parameters and unpause
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

        for i in range(0, 1000):
            self._rviz_pub.publish(initial)
        time.sleep(1)

        # set goal
        msg = PodiMoveBaseActionGoal()
        msg.goal.target_poses.header.frame_id = "map"
        
        for goal in finish:
            if planner == "robot_only":
                new_xy = c.human_to_robot(goal["x"], goal["y"], goal["angle"])
                new_pose = Pose()
                new_pose.position.x = new_xy[0]
                new_pose.position.y = new_xy[1]
                new_pose.position.z = 0
            elif planner == "coupled":
                new_pose = Pose()
                new_pose.position.x = goal["x"] #+ 0.32 * math.cos(finish["x"] * math.pi / 180)
                new_pose.position.y = goal["y"] #+ 0.32 * math.sin(finish["y"] * math.pi / 180)
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

        time.sleep(3)
        rospy.loginfo(new_pose)
        for j in range (0, 10):
            self._goal_pub.publish(msg)
    
    def _kill(self):
        self._child.send_signal(signal.SIGINT)
        self._child.wait()

    def _wait(self):
        while not rospy.is_shutdown():
            if self._restart:
	            break


if __name__ == '__main__':
    rospy.init_node("simulate")

    rospack = rospkg.RosPack()
    directory = rospack.get_path("p3dx_2dnav")
    jsonfile = "locations.json"
    filepath = directory + "/json/" + jsonfile

    with open(filepath) as f:
        data = json.load(f)
    
    s = Simulation()

    # example usage
    for start in data["trajectory"]["robot_only"]["starts"]:
        if data["trajectory"]["robot_only"]["starts"].index(start) != 0:
            s._wait()
        s._simulate("robot_only", start, data["trajectory"]["robot_only"]["goals"])
        s._wait()
        s._kill()
    
