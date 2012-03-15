#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <stdio.h>
#include <stdlib.h>
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class MoveOmniBase {
 public:
  ros::NodeHandle nh;
  MoveOmniBase(std::string name, tf::TransformListener& tf);
  void BaseLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);
  geometry_msgs::PoseStamped goalToLocalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
  double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

  //virtual ~MoveOmniBase();

  tf::TransformListener& tf_;
  MoveBaseActionServer* as_;

  //ros::ServiceServer align_save_srv_;
  //message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  //tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  


  ros::Publisher vel_pub_;
  ros::Publisher action_goal_pub_;

  ros::Subscriber goal_sub_;
  ros::Subscriber base_scan_sub_;
  bool enableLaser;
  bool front_collision;
  bool left_collision;
  bool right_collision;
};
