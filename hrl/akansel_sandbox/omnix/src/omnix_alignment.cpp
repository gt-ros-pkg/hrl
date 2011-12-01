#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <scan_mapper/canonical_scan.h>

class OmnixAlignment{
private:
  std::map<std::string,sensor_msgs::LaserScan> location_scans;
  
  sensor_msgs::LaserScan prev_scan_;

  bool align_active_;
  std::string goal_name_;

  scan_tools::CanonicalScan canonicalScan;

public:
  ros::NodeHandle n_;
  tf::TransformListener tf_listener_;

  ros::ServiceServer align_save_srv_;
  ros::Subscriber laser_sub_;
  
  OmnixAlignment() :
    n_("~"),
    tf_listener_(ros::Duration(60.0))
  {
    align_active_ = false;

    laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan",1,boost::bind(&OmnixAlignment::laserScanCallback,this,_1));

    align_save_srv_ = n_.advertiseService("save_location",&OmnixAlignment::alignSaveCallback,this);
  }
  
  bool OmnixAlignment::alignSaveCallback(omnix::AlignSave::Request &req, omnix::AlignSave::Response &res)
  {
    //save previous laser scan as this string
    location_scans[req.location_name] = prev_scan_;
    return true;
  }

  bool OmnixAlignment::alignCallback(omnix::Align::Request &req, omnix::Align::Response &res)
  {
    //Check if we know this location
    if(location_scans.find(res.location_name) == location_scans.end()){
      ROS_ERROR("OmnixAlignment: Requested key not known!");
      return false;
    }
    
    //Set new goal
    goal_name_ = req.location_name;

    //Set alignment active
    align_active_ = true;

    return true;
  }

  void OmnixAlignment::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
  {
    LDP prev_ldp;
    LDP curr_ldp;
    
    if(align_active){
      //ICP to goal scan
      canonicalScan.laserScanToLDP(prev_scan_,prev_ldp);
      canonicalScan.laserScanToLDP(scan_msg,curr_ldp);
     
      tf::Transform init_pose = btTransform(tf::createQuaternionFromYaw(0.0),btVector3(0.0,0.0,0.0));
      gtsam::Pose2 output_pose;
      gtsam::noiseModel::Gaussian::shared_ptr noise_model;

      bool worked = canonicalScan.processScan(curr_ldp,prev_ldp,
					      init_pose,
					      output_pose,
					      noise_model);

      
      if(!worked){
	ROS_INFO("ICP fail!");
      } else {
	ROS_INFO("Pose: %lf %lf %lf",output_pose.x(),output_pose.y(),output_pose.z());
	  
	  
	//Tform to base link
	tf::StampedTransform laser_to_base;
	try{
	  tf_listener_->waitForTransform(scan_msg.header.frame_id, "/base_link",scan_msg.header.stamp,ros::Duration(0.6));
	  tf_listener_->lookupTransform(scan_msg.header.frame_id, "/base_link",scan_msg.header.stamp,laser_to_base);
	} catch(tf::TransformException ex){
	  ROS_ERROR("Omnix Alignment: TF Fail!");
	}
	
	
	//Calc goal vec
	
	//Send motion to Omnix
      
      }
      
    }
    
    prev_scan_ = scan_msg;
  }

};
