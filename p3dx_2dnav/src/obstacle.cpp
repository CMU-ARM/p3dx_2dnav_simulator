#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iterator>
#include <vector>

// some definitions of functions
#define MAP_INDEX(map, i, j) ((i) + (j) * map.size_x)
#define MAP_WXGX(map, i) (map.origin_x + (i - map.size_x / 2) * map.scale)
#define MAP_WYGY(map, j) (map.origin_y + (j - map.size_y / 2) * map.scale)

// information of the map
struct map_inf {
    double size_x;
    double size_y;
    double scale;
    double origin_x;
    double origin_y;
};


std::pair<double, double> poseToMap(geometry_msgs::PoseStamped & pt,
                                    nav_msgs::OccupancyGrid & map){

    std::pair<double, double> result(0,0);
    // remapping information in info variable
    map_inf map_info;
    map_info.size_x = map.info.width;
    map_info.size_y = map.info.height;
    map_info.scale = map.info.resolution;
    map_info.origin_x = map.info.origin.position.x + (map_info.size_x / 2) * map_info.scale;
    map_info.origin_y = map.info.origin.position.y + (map_info.size_y / 2) * map_info.scale;

    // map (world reference)
    double gt_x = map.info.origin.position.x;
    double gt_y = map.info.origin.position.y;

    tf::TransformListener tf_l(ros::Duration(10));
    geometry_msgs::PoseStamped new_pt;
    bool success = false;

    // next line to avoid tf extrapolation into the past (not good code)
    map.header.stamp = pt.header.stamp = ros::Time(0);

    // transform in case the pose to evaluate and the point are not in the same
    // reference frame
    try{
        tf_l.waitForTransform(map.header.frame_id,
                              pt.header.frame_id, ros::Time(0), ros::Duration(1));
        tf_l.transformPose(map.header.frame_id, pt, new_pt);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    // robot in map frame reference
    double pt_x = new_pt.pose.position.x;
    double pt_y = new_pt.pose.position.y;
    double pt_th = tf::getYaw(new_pt.pose.orientation);

    // variables to keep the distance
    double dist_sq, min_dist = 100000;

    double w_x, w_y;  // world cordinates
    double d_x, d_y;  // saved world cordinates
    ros::Time start = ros::Time::now();
    // more efficient first with heigh and then width because map information
    // is in row-major order
    for (std::size_t j=0; j < map_info.size_y; j++) {
        for (std::size_t i=0; i < map_info.size_x; i++) {
            if(map.data[MAP_INDEX(map_info, i, j)]==100){
                // convert to world position
                w_x = MAP_WXGX(map_info, i);
                w_y = MAP_WYGY(map_info, j);
                dist_sq = pow(w_x-pt_x,2)+pow(w_y-pt_y,2);
                if (dist_sq<min_dist){
                    min_dist = dist_sq;
                    d_x = w_x;
                    d_y = w_y;
                }
            }
        }
    }
    min_dist = sqrt(min_dist);

    // given the position of the obstacle, get the angle in the robot reference
    double angle;
    angle = std::atan2(d_y-pt_y, d_x-pt_x)-pt_th;

    // debug information
    //ROS_INFO_STREAM("Finding in "<< ros::Time::now()-start<<"s");
    //ROS_INFO_STREAM("Min. Distance and angle: "<< min_dist << "," <<angle*180/M_PI);
    //ROS_INFO_STREAM("Robot is at position "<< pt_x <<","<<pt_y<<","<<pt_th*180/M_PI);
    //ROS_INFO_STREAM("Obstacle is at position "<< d_x <<","<<d_y);
    result = std::make_pair(min_dist, angle);
    return result;
}

using namespace std;
class consolidate{
	public:
		nav_msgs::OccupancyGrid last_map;
		geometry_msgs::PoseStamped last_pose;
    bool receivedFirstCmdVel;

    consolidate();
		void costmap_cb(const nav_msgs::OccupancyGrid & msg);
    void cmd_vel_cb(const geometry_msgs::Twist & msg);
		void pose_cb(const geometry_msgs::PoseStamped & msg);
};

consolidate::consolidate() {
  receivedFirstCmdVel = false;
}

void consolidate::costmap_cb(const nav_msgs::OccupancyGrid & msg)
{
	consolidate::last_map = msg;
}

void consolidate::cmd_vel_cb(const geometry_msgs::Twist & msg) {
  if (msg.linear.x != 0.0 || msg.angular.z != 0.0) {
    receivedFirstCmdVel = true;
  }
}

void consolidate::pose_cb(const geometry_msgs::PoseStamped & msg)
{
	consolidate::last_pose = msg;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "obstacle");
  ros::NodeHandle n;
  consolidate c;

  ros::Subscriber csub = n.subscribe("/podi_move_base/global_costmap/costmap", 1000, &consolidate::costmap_cb, &c);
  ros::Subscriber psub = n.subscribe("/coupling_model_node/human_position", 1000, &consolidate::pose_cb, &c);
  ros::Subscriber cmd_vel_sub = n.subscribe("/p3dx/cmd_vel", 1000, &consolidate::cmd_vel_cb, &c);
  ros::Publisher dpub = n.advertise<std_msgs::Float64>("obstacle_dist", 1000);

  ros::Duration(7).sleep();
  ros::Rate r(5); // the frequency, in Hertz, at which to sample obstacle cost
  while(ros::ok()) {
    if (c.receivedFirstCmdVel) {
      std::pair <double, double> x = poseToMap(c.last_pose, c.last_map);
    	std_msgs::Float64 msg;
    	msg.data = x.first;
    	dpub.publish(msg);

      r.sleep(); // sleep until the appropriate number of seconds (based on the frequency) have elapsed since the last sleep
    }
  	ros::spinOnce();
  }
  return 0;
}
