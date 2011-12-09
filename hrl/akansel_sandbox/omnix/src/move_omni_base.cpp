#include <omnix/move_omni_base.h>
#include <iostream>
#include <stdio.h>

MoveOmniBase::MoveOmniBase(std::string name, tf::TransformListener& tf) :
  tf_(tf),
  as_(NULL),
enableLaser(true){
  ROS_INFO("Starting move omnibase");
  as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveOmniBase::executeCb, this, _1), false);

	left_collision=false;
	right_collision=false;
	front_collision=false;
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //Set up move base action
  ros::NodeHandle action_nh("move_base");
  action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

  //Set up simple goal subscriber for Rviz
  ros::NodeHandle simple_nh("move_omni_base_simple");
  goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveOmniBase::goalCB, this, _1));
  base_scan_sub_ = simple_nh.subscribe<sensor_msgs::LaserScan>("/base_scan",1,boost::bind(&MoveOmniBase::BaseLaserCB, this, _1));
  as_->start();
  ROS_INFO("Move omni base started.");
}

void MoveOmniBase::BaseLaserCB(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
if(enableLaser)
{
sensor_msgs::PointCloud cloud;
laser_geometry::LaserProjection projector_;
projector_.projectLaser(*scan_in,cloud);
//ROS_INFO("LASER %d PTS",cloud.points.size());
front_collision=false;
left_collision=false;
right_collision=false;
for(unsigned int i=0;i<cloud.points.size();i++)
{
geometry_msgs::Point32 pt=cloud.points[i];
	if(pt.x<0.09 && pt.x>0 && ((pt.y<0.3 && pt.y>0)||(pt.y>-0.3 && pt.y<0)) )
	{
	front_collision=true;
	}
	if(pt.y>0.2 && pt.y<0.36 && pt.x<0.05 && pt.x>-0.6)
	{
	left_collision=true;
	}
	if(pt.y>-0.36 && pt.y<-0.2 && pt.x<0.05 && pt.x>-0.6)
	{
	right_collision=true;
	}
}
//ROS_INFO("COLL: %d %d %d",left_collision,front_collision,right_collision);
}
return;
}

void MoveOmniBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  ROS_INFO("In goal callback!");
  move_base_msgs::MoveBaseActionGoal action_goal;
  action_goal.header.stamp = ros::Time::now();
  action_goal.goal.target_pose = *goal;
  action_goal_pub_.publish(action_goal);
}

geometry_msgs::PoseStamped MoveOmniBase::goalToLocalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
{
  tf::Stamped<tf::Pose> goal_pose, local_pose;
  poseStampedMsgToTF(goal_pose_msg, goal_pose);
  
  goal_pose.stamp_ = ros::Time();
  
  try{
    tf_.transformPose("/base_link", goal_pose, local_pose);
  } catch (tf::TransformException& ex) {
    ROS_WARN("move_omni_base: TF fail.");
    return goal_pose_msg;
  }
  
  geometry_msgs::PoseStamped local_pose_msg;
  tf::poseStampedTFToMsg(local_pose, local_pose_msg);
  return local_pose_msg;
}

double MoveOmniBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
  return sqrt((p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x)
	      + (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y));
}

void MoveOmniBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
{
  //ROS_INFO("in executeCB");
  bool at_goal = false;
	bool coll=false;
  double goal_tolerance = 0.03;//meters
  double goal_tolerance_rot = 0.11;
  double xy_scale = 0.2;
  double rot_scale = 1.4; //0.1
  ros::Rate r(30);
	
	double min_rot_vel=0.12;
	
  geometry_msgs::PoseStamped base = geometry_msgs::PoseStamped();
  enableLaser=true;
  
  while(!(as_->isPreemptRequested()) && (ros::ok()) && (!at_goal)){
    //Get the goal location in the base frame
    
		geometry_msgs::PoseStamped goal = goalToLocalFrame(move_base_goal->target_pose);
    double goal_yaw = tf::getYaw(goal.pose.orientation);
    
    //Check if we're close enough
	//std::cout<<"(distance(goal,base): "<<distance(goal,base)<<std::endl;
	//std::cout<<"fabs(goal_yaw): "<<fabs(goal_yaw)<<std::endl;

bool yaw_satisfied;
if(fabs(goal_yaw) < goal_tolerance_rot)
yaw_satisfied=true;
else
yaw_satisfied=false;

bool pos_satisfied;
if(distance(goal,base) <= goal_tolerance)
pos_satisfied=true;
else
pos_satisfied=false;
	
	if(yaw_satisfied && pos_satisfied)
    {
      at_goal = true;
      break;
    } 
    else 
    {      
      //Calculate a velocity
      double xvel, yvel = 0.0;
      tf::Vector3 vel_vec = tf::Vector3(goal.pose.position.x,
					goal.pose.position.y,
					0.0);
      vel_vec.normalize();
      xvel = vel_vec.getX();
      yvel = vel_vec.getY();

      //ROS_INFO("Goal yaw: %lf",goal_yaw);
      
      geometry_msgs::Twist cmd_vel;
			if(pos_satisfied)
			{
			cmd_vel.linear.x =0.0;
			cmd_vel.linear.y =0.0;
			}
			else
			{
      cmd_vel.linear.x = xvel * xy_scale;
      cmd_vel.linear.y = yvel * xy_scale;
			}
		if(yaw_satisfied)
		{
		cmd_vel.angular.z=0;
		}
		else
		{
      double rot_vel =  (goal_yaw / M_PI) * rot_scale;

			if(rot_vel<0.0) //rot_vel negative
			{
				if(rot_vel>-min_rot_vel)
				{
					rot_vel=-min_rot_vel;
				}
			}
			else //rot_vel positive
			{
				if(rot_vel<min_rot_vel)
				{
				rot_vel=min_rot_vel;
				}
			}
			
      cmd_vel.angular.z = rot_vel;      
		}
		
		if(cmd_vel.linear.x>0.001 && front_collision)
		{
		coll=true;
		}
		if(cmd_vel.linear.y>0.001) //check left
		{
			if(left_collision)
			{
			coll=true;
			}
		}
		else if(cmd_vel.linear.y<-0.001) //check right
		{
			if(right_collision)
			{
			coll=true;
			}
		}
	if(coll)
	{
	cmd_vel.linear.x=0;
	cmd_vel.linear.y=0;
	cmd_vel.linear.z=0;
	cmd_vel.angular.z=0;
	vel_pub_.publish(cmd_vel);
break;
	}
	else
	{
				vel_pub_.publish(cmd_vel);
	}
									
      r.sleep();
    }
  }

  if(at_goal)
  {
    as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
  }
	else
	{
		as_->setAborted(move_base_msgs::MoveBaseResult(), "Goal aborted.");
	}
	enableLaser=false;
	front_collision=false;
	left_collision=false;
	right_collision=false;
  return;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "move_omni_base_node");
  tf::TransformListener tf(ros::Duration(10));
  
  MoveOmniBase move_omni_base("move_omni_base", tf);
  
  ros::spin();
  
  return(0);
}
