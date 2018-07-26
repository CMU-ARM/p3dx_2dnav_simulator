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

namespace podi_navigation_helpers {

class RobotToGroundTruthNode
{
public:
  RobotToGroundTruthNode();
  ~RobotToGroundTruthNode();
private:
  ros::NodeHandle private_nh;
	std::thread updateOdomToBaseLinkTransformThread;

	tf::TransformListener tf_;
  tf::TransformBroadcaster tf_broadcaster_;

  tf::Stamped<tf::Pose> odomToBaseLinkTransform;
	std::mutex odomToBaseLinkTransformMutex;

  ros::Subscriber groundTruthSub_;

	void updateOdomToBaseLinkTransform();
	void groundTruthCB(const nav_msgs::Odometry::ConstPtr &msg);
};

RobotToGroundTruthNode::RobotToGroundTruthNode() {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

	tf::TransformListener tf_(ros::Duration(10));

	groundTruthSub_ = nh.subscribe("/p3dx/base_pose_ground_truth", 1000, &RobotToGroundTruthNode::groundTruthCB, this);

  // Start the thread that will keep updating the odomToBaseLinkTransform
	updateOdomToBaseLinkTransformThread = std::thread(&RobotToGroundTruthNode::updateOdomToBaseLinkTransform, this);
}

RobotToGroundTruthNode::~RobotToGroundTruthNode() {
  updateOdomToBaseLinkTransformThread.join();
}

void RobotToGroundTruthNode::updateOdomToBaseLinkTransform() {
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
			tf_.waitForTransform("/odom", base_link_pose.frame_id_, base_link_pose.stamp_, ros::Duration(1.0));
			tf_.transformPose("/odom", base_link_pose, base_link_in_odom_frame_pose);
    } catch (tf::TransformException ex) {
      ROS_ERROR("RobotToGroundTruthNode::updateOdomToBaseLinkTransform %s", ex.what());
      loop_rate.sleep();
			continue;
    }

		std::unique_lock<std::mutex> odomToBaseLinkTransformLock(odomToBaseLinkTransformMutex);
    odomToBaseLinkTransform = base_link_in_odom_frame_pose;
    odomToBaseLinkTransformLock.unlock();
	}
}

void RobotToGroundTruthNode::groundTruthCB(const nav_msgs::Odometry::ConstPtr &msg) {
    //ROS_WARN("Got ground truth message");
  double pose_roll, pose_pitch, pose_yaw;
  double otbl_roll, otbl_pitch, otbl_yaw;
  tf::Transform otbl_t, t_otbl;
  tf::Stamped<tf::Pose> groundTruth;
	geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  tf::poseStampedMsgToTF(pose, groundTruth);
  tf::Matrix3x3(groundTruth.getRotation()).getRPY(pose_roll, pose_pitch, pose_yaw);
  tf::Transform transform;
  tf::Quaternion q;
  std::unique_lock<std::mutex> odomToBaseLinkTransformLock(odomToBaseLinkTransformMutex);
  //ROS_WARN("GT x: %f y: %f z: %f", groundTruth.getOrigin().getX(), groundTruth.getOrigin().getY(), groundTruth.getOrigin().getZ());
  //ROS_WARN("OTBL x: %f y: %f z: %f", odomToBaseLinkTransform.getOrigin().getX(), odomToBaseLinkTransform.getOrigin().getY(), odomToBaseLinkTransform.getOrigin().getZ());
  //transform = odomToBaseLinkTransform * groundTruth;
  transform = groundTruth * odomToBaseLinkTransform.inverse();
  otbl_t.mult(odomToBaseLinkTransform, transform);
  t_otbl.mult(transform, odomToBaseLinkTransform);
  //ROS_WARN("OTBL * T %f", otbl_t.getOrigin().getX());
  //ROS_WARN("T * OTBL %f", t_otbl.getOrigin().getX());
  //transform.setOrigin(tf::Vector3(groundTruth.getOrigin().getX() - odomToBaseLinkTransform.getOrigin().getX(), groundTruth.getOrigin().getY() - odomToBaseLinkTransform.getOrigin().getY(), groundTruth.getOrigin().getZ() - odomToBaseLinkTransform.getOrigin().getZ()));
  //tf::Matrix3x3(odomToBaseLinkTransform.getRotation()).getRPY(otbl_roll, otbl_pitch, otbl_yaw);
  //q.setRPY(pose_roll - otbl_roll, pose_pitch - otbl_pitch, pose_yaw - otbl_yaw);
  //transform.setRotation(q);
  //ROS_WARN("FINAL x: %f y: %f z: %f", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
  
  odomToBaseLinkTransformLock.unlock();
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time(pose.header.stamp), "new_map", "odom"));
}

} // end namespace

int main(int argc, char **argv) {
	ros::init(argc, argv, "transform_robot_to_ground_truth");
  podi_navigation_helpers::RobotToGroundTruthNode rgtn;
	ros::spin();

	return 0;
}
