#include <iostream>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <string>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

namespace podi_navigation_helpers {

class lmsToBaseNode
{
public:
  lmsToBaseNode();
  ~lmsToBaseNode();
private:
  ros::NodeHandle private_nh;
	std::thread updatelmsToBaseTransformThread;

	tf::TransformListener tf_;
  tf::TransformBroadcaster tf_broadcaster_;

  tf::Stamped<tf::Pose> lmsToBaseTransform;
	std::mutex lmsToBaseTransformMutex;

  ros::Subscriber lmsToBaseSub_;

	void updatelmsToBaseTransform();
	void groundTruthCB(const sensor_msgs::LaserScan::ConstPtr &msg);
};

lmsToBaseNode::lmsToBaseNode() {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

	tf::TransformListener tf_(ros::Duration(10));

	lmsToBaseSub_ = nh.subscribe("/p3dx/laser/scan", 1000, &lmsToBaseNode::groundTruthCB, this);

  // Start the thread that will keep updating the lmsToBaseTransform
	updatelmsToBaseTransformThread = std::thread(&lmsToBaseNode::updatelmsToBaseTransform, this);
}

lmsToBaseNode::~lmsToBaseNode() {
  updatelmsToBaseTransformThread.join();
}

void lmsToBaseNode::updatelmsToBaseTransform() {
  // ros::Rate doesn't work across threads, that's why I have to create it in
  // this function and not the initialization function
  double loop_freq;
  std::tuple<int, int, int> oldHumanPos, newHumanPos;
  private_nh.param("update_human_pose_freq", loop_freq, 100.0); // Hz
  ros::Rate loop_rate(loop_freq);
	while (!ros::isShuttingDown()) {
	//ROS_WARN("In odom to base_link loop");
    loop_rate.sleep();
		// transfrom the 0 vector in \base_link to \odom
		tf::Stamped<tf::Pose> base_link_pose, base_link_in_odom_frame_pose;
    base_link_pose.frame_id_ = "/base_link";
    base_link_pose.stamp_ = ros::Time::now();
    base_link_pose.setOrigin(tf::Vector3(0, 0, 0));
		base_link_pose.setRotation(tf::Quaternion(0, 0, 0, 1));

		try {
			tf_.waitForTransform("/p3dx/laser/scan", base_link_pose.frame_id_, base_link_pose.stamp_, ros::Duration(1.0));
			tf_.transformPose("/p3dx/laser/scan", base_link_pose, base_link_in_odom_frame_pose);
    } catch (tf::TransformException ex) {
      ROS_ERROR("lmsToBaseNode::updatelmsToBaseTransform %s", ex.what());
      loop_rate.sleep();
			continue;
    }

		std::unique_lock<std::mutex> lmsToBaseTransformLock(lmsToBaseTransformMutex);
    lmsToBaseTransform = base_link_in_odom_frame_pose;
    lmsToBaseTransformLock.unlock();
	}
}

void lmsToBaseNode::groundTruthCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
    //ROS_WARN("Got ground truth message");
  double pose_roll, pose_pitch, pose_yaw;
  //double otbl_roll, otbl_pitch, otbl_yaw;
  //tf::Transform otbl_t, t_otbl;
  tf::Stamped<tf::Pose> groundTruth;
	geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  //pose.pose = msg->pose.pose;
  //tf::poseStampedMsgToTF(pose, groundTruth);
  //tf::Matrix3x3(groundTruth.getRotation()).getRPY(pose_roll, pose_pitch, pose_yaw);
  tf::Transform transform;
  //tf::Quaternion q;
  std::unique_lock<std::mutex> lmsToBaseTransformLock(lmsToBaseTransformMutex);
  transform.setOrigin(tf::Vector3(0, 0, 0));
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  //ROS_WARN("GT x: %f y: %f z: %f", groundTruth.getOrigin().getX(), groundTruth.getOrigin().getY(), groundTruth.getOrigin().getZ());
  //ROS_WARN("OTBL x: %f y: %f z: %f", lmsToBaseTransform.getOrigin().getX(), lmsToBaseTransform.getOrigin().getY(), lmsToBaseTransform.getOrigin().getZ());
  //transform = lmsToBaseTransform * groundTruth;
  //transform = groundTruth * lmsToBaseTransform.inverse();
  //otbl_t.mult(lmsToBaseTransform, transform);
  //t_otbl.mult(transform, lmsToBaseTransform);
  //ROS_WARN("OTBL * T %f", otbl_t.getOrigin().getX());
  //ROS_WARN("T * OTBL %f", t_otbl.getOrigin().getX());
  //transform.setOrigin(tf::Vector3(groundTruth.getOrigin().getX(), groundTruth.getOrigin().getY(), groundTruth.getOrigin().getZ());
  //tf::Matrix3x3(lmsToBaseTransform.getRotation()).getRPY(otbl_roll, otbl_pitch, otbl_yaw);
  //q.setRPY(pose_roll, pose_pitch, pose_yaw);
  //transform.setRotation(q);
  //ROS_WARN("FINAL x: %f y: %f z: %f", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

  lmsToBaseTransformLock.unlock();
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time(pose.header.stamp), "lms100", "base_link"));
}

} // end namespace

int main(int argc, char **argv) {
	ros::init(argc, argv, "transform_lms100_to_base_link");
  podi_navigation_helpers::lmsToBaseNode ltbl;
	ros::spin();

	return 0;
}
