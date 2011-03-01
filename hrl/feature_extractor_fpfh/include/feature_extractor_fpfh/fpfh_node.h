#ifndef FPFHNODE_
#define FPFHNODE_
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"                                                                   

class FPFHNode                                                                                         
{       
    protected:
        ros::NodeHandle n_;
        ros::Publisher hist_publisher;
        ros::Subscriber points_subscriber;

    public:
        FPFHNode(ros::NodeHandle &n);
        void points_cb(const sensor_msgs::PointCloud2::ConstPtr &input_cloud);
}; 
#endif  //#ifndef FPFHNODE_

