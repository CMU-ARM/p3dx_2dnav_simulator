#!/usr/bin/env python
import rospy
import rosnode
import rosgraph
import sys
import argparse
import os
import time
from std_msgs.msg import Bool

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

# checks if gazebo has died prematurely, and if so, messages for kill and restart
class Pid:
    def __init__(self):
        self._pid_pub = rospy.Publisher("/gazebo_running", Bool, queue_size=1)

    def _pid_running(self, pid):
        try:
            os.kill(pid,0)
        except OSError:
            return False
        else:
            return True

    def check(self):
        while not rospy.is_shutdown():
            ID = '/rosnode'
            master = rosgraph.Master(ID, master_uri="http://tbw:11311/")

            node_api = rosnode.get_api_uri(master, '/gazebo')
            msg = Bool()

            try:
                node = ServerProxy(node_api)
                pid = rosnode._succeed(node.getPid(ID))
                if not self._pid_running(pid):
                    msg.data = False
                    self._pid_pub.publish(msg)
                else:
                    msg.data = True
                    self._pid_pub.publish(msg)
            except TypeError:
                os.system("killall gzserver")
                msg.data = False
                self._pid_pub.publish(msg)
                exit()

if __name__ == "__main__":
    rospy.init_node("alive")
    p = Pid()
    time.sleep(10)
    p.check()
