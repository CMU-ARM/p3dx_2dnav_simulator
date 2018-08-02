#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Transform:
    def __init__(self):
        self.sub = rospy.Subscriber('/p3dx/laser/scan', LaserScan, self.handle_transform, queue_size=2)
        self.pub = rospy.Publisher('/p3dx/laser/new_scan', LaserScan, queue_size=1)
        #self.sub1 = rospy.Subscriber('/p3dx/laser/scan', LaserScan, self.handle, queue_size=2)
        #self.pub1 = rospy.Publisher('/tmp', LaserScan, queue_size=1)
    
    def handle_transform(self, msg):
        msg.header.frame_id = "new_laser"
        self.pub.publish(msg)
                     
    '''def handle(self, msg):
        t1 = tf.TransformBroadcaster()
        t1.sendTransform((0, 0, 0),
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     "lms100",
                     "base_link")
        new_msg = msg
        new_msg.header.frame_id = "base_link"
        self.pub1.publish(new_msg)'''

if __name__ == '__main__':
    rospy.init_node('transformation')
    t = Transform()
    rospy.spin()
    #listener = tf.TransformListener()
    #t = tf.TransformBroadcaster()
    
    '''while not rospy.is_shutdown():
        if listener.frameExists("/base_link") and listener.frameExists("/map"):
            try:
                time1 = tf.getLatestCommonTime("/base_link", "/odom")
                (tr1, rot1) = listener.lookupTransform('/base_link', '/odom', time1)
                time2 = tf.getLatestCommonTime("/odom", "/map")
                (tr2, rot2) = listener.lookupTransform('/odom', '/map', time2)
                #t.sendTransform(tr1 + tr2, rot1 + rot2, rospy.Time.now(), '/base_link', '/map')
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Bad transform")'''
                
    '''try:
    time = listener.getLatestCommonTime("/base_link", "/map")
    (tr1, rot1) = listener.lookupTransform('/base_link', '/map', time)
    print position, quaternion
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    rospy.logerr("Bad transform")'''
    
