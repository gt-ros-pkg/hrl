/*
 * omnix_ps3joy.cpp
 *
 *  Created on: Jun 3, 2011
 *      Author: atrevor
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <omnix/buttons.h>

class TeleopOmnix
{
public:
  TeleopOmnix();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

TeleopOmnix::TeleopOmnix():
    nh_("~"),
    l_scale_(0.4),
    a_scale_(0.4)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &TeleopOmnix::joyCallback, this);

  ROS_INFO("TeleopOmnix: Teleoperation and Emergency Stop Interface active!");
}

void saturate_control(double& val, double max){
  if (max < 0.0)
    throw -1;
  if (val > max) val = max;
  if (val < -max) val = -max;
}

void TeleopOmnix::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  double angvel = a_scale_ * joy->axes[PS3_RIGHT_STICK_LR];
  saturate_control(angvel, 0.4);
  vel.angular.z = angvel;
  double linvel_x = l_scale_ * joy->axes[PS3_LEFT_STICK_UD];
  saturate_control(linvel_x, 0.6);
  vel.linear.x = linvel_x;
  double linvel_y = l_scale_ * joy->axes[PS3_LEFT_STICK_LR];
  saturate_control(linvel_y, 0.6);
  vel.linear.y = linvel_y;

  vel_pub_.publish(vel);
  
}

int main(int argc, char** argv)
{
 ros::init(argc, argv, "omnix_ps3joy");
 TeleopOmnix teleop_omnix;

 ros::spin();
 return 0;
}

