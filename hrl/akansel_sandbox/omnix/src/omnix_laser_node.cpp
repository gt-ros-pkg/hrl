#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> LaserSyncPolicy;

class OmnixLaserMergeNode
{
public:
  ros::NodeHandle n_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser1_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser2_sub_;
  tf::TransformListener tf_listener_;

  OmnixLaserMergeNode():
    n_("~"),
    tf_listener_(ros::Duration(60.0))
  {
    message_filters::Subscriber<sensor_msgs::LaserScan>  laser1_sub_(n_,"/utm30_1", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan>  laser2_sub_(n_,"/utm30_2", 1);

    message_filters::Synchronizer<LaserSyncPolicy> sync(LaserSyncPolicy(10), laser1_sub_, laser2_sub_);
    sync.registerCallback(boost::bind(&this->doubleLaserCallback, _1, _2));
    
  }

  void doubleLaserCallback(const sensor_msgs::LaserScan& scan1, const sensor_msgs::LaserScan& scan2)
  {
    //transform both to the base frame, and republish this to the laser line extractor.
    tf::StampedTransform laser1_to_base;
    tf::StampedTransform laser2_to_base;
    try{
      tf_listener_->waitForTransform("/laser1", "/base_link",scan1.header.stamp, ros::Duration(0.6));
      tf_listener_->lookupTransform("/laser1","/base_link",scan1.header.stamp, laser1_to_base);
      tf_listener_->waitForTransform("/laser2", "/base_link",scan2.header.stamp, ros::Duration(0.6));
      tf_listener_->lookupTransform("/laser2","/base_link",scan2.header.stamp, laser2_to_base);
    } catch(tf::TransformException ex){
      ROS_ERROR("Laser Merge Callback failure: %s",ex.what());
    }
    
    
  }
};
