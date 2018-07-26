#include <iostream>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <string>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdio>
#include <ctime>
#include <Eigen/Core>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"

namespace convert_pose {

  class ConvertPoseNode
  {
  public:
    ConvertPoseNode();
    ~ConvertPoseNode();
  private:
    ros::NodeHandle private_nh;
  	std::thread ConvertPoseThread;
  	tf::TransformListener tf_;
    ros::Publisher pub;
    // ros::Subscriber sub;
  	void ConvertPose();
    void callback(const nav_msgs::Odometry &msg);
  };

  ConvertPoseNode::ConvertPoseNode() {
    ros::NodeHandle private_nh("~");
    pub = private_nh.advertise<geometry_msgs::PoseStamped>("pose_converted", 1);
    // sub = private_nh.subscribe("/p3dx/base_pose_ground_truth", 1, &ConvertPoseNode::callback, this);
    // Start the thread that will keep updating the pose conversion
    ConvertPoseThread = std::thread(&ConvertPoseNode::ConvertPose, this);
  }

  ConvertPoseNode::~ConvertPoseNode() {
    ConvertPoseThread.join();
  }

  void ConvertPoseNode::ConvertPose() {
    // ros::Rate doesn't work across threads, that's why I have to create it in
    // this function and not the initialization function
    ros::Rate loop_rate(10);
    tf::Stamped<tf::Pose> base_pose, robot_pose;
    geometry_msgs::PoseStamped tmp;
  	while (!ros::isShuttingDown()) {
        loop_rate.sleep();

  		// transfrom the 0 vector in \base_link to \map

        base_pose.frame_id_ = "/base_link";
        base_pose.stamp_ = ros::Time::now();
        base_pose.setOrigin(tf::Vector3(0, 0, 0));
        base_pose.setRotation(tf::Quaternion(0, 0, 0, 1));

  		  try {
  			     tf_.waitForTransform("/map", base_pose.frame_id_, base_pose.stamp_, ros::Duration(1.0));
  			     tf_.transformPose("/map", base_pose, robot_pose);
             tf::poseStampedTFToMsg(robot_pose, tmp);
             pub.publish(tmp);
        } catch (tf::TransformException ex) {
             ROS_ERROR("ConvertPoseNode::ConvertPose %s", ex.what());
             loop_rate.sleep();
  			     continue;
        }
  	}
  }
} // end namespace

int main(int argc, char **argv) {
	ros::init(argc, argv, "convert_pose_node");
  convert_pose::ConvertPoseNode cpn;
	ros::spin();
	return 0;
}
