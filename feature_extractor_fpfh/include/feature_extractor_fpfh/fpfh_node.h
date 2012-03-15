#ifndef FPFHNODE_
#define FPFHNODE_
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"                                                                   
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "feature_extractor_fpfh/FPFHHist.h"
#include "feature_extractor_fpfh/SubsampleCalc.h"

class FPFHNode
{
    protected:
        ros::NodeHandle n_;
        ros::Publisher hist_publisher;
        ros::ServiceServer fpfh_service;
        ros::ServiceServer subsample_service;

    public:
        FPFHNode(ros::NodeHandle &);

        void message_cb(const sensor_msgs::Image::ConstPtr& , const sensor_msgs::PointCloud2::ConstPtr& );

        void process_point_cloud(const sensor_msgs::PointCloud2::ConstPtr&, feature_extractor_fpfh::FPFHHist &);

        bool fpfh_srv_cb(feature_extractor_fpfh::FPFHCalc::Request &, feature_extractor_fpfh::FPFHCalc::Response &);

        bool subsample_srv_cb(feature_extractor_fpfh::SubsampleCalc::Request &, feature_extractor_fpfh::SubsampleCalc::Response &);
};

#endif  //#ifndef FPFHNODE_

