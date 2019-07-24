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
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptySrv)
        self._unpause = rospy.ServiceProxy("/gazebo/unpause_physics", EmptySrv)
        self._reset = rospy.ServiceProxy("/gazebo/reset_simulation", EmptySrv)
        self._rviz_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=2, latch=True)
        self._goal_pub = rospy.Publisher("/podi_move_base/goal", PodiMoveBaseActionGoal, queue_size=2)
        self._result_sub = rospy.Subscriber("podi_move_base/result", PodiMoveBaseActionResult, self._result_cb, queue_size=1)
        #self._endgoal_pub = rospy.Publisher("/endgoals", PoseArray, queue_size=2)
        self._endgoal_pub = rospy.Publisher("/coupled_planner/planner/robot_traj_goal", PoseArray, queue_size=2)
        self._restart = False

    def _result_cb(self, msg):
        if msg.status.text == "Goal reached.":
            self._restart = True

    def _simulate(self, planner, start=None, finish=None):
        # set goal
        msg = PodiMoveBaseActionGoal()
        msg.goal.target_poses.header.frame_id = "map"
        c = Convert()
        end_goals = PoseArray()

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
            end_goals.header.frame_id = "map"
            end_goals.poses.append(new_pose)

        time.sleep(3)
        rospy.logerr(msg)
        for j in range (0, 10):
            self._endgoal_pub.publish(end_goals)
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
