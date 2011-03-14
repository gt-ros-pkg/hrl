#ifndef FPFHNODE_
#define FPFHNODE_
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"                                                                   
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "feature_extractor_fpfh/FPFHHist.h"

class FPFHNode
{
    protected:
        ros::NodeHandle n_;
        ros::Publisher hist_publisher;
        ros::ServiceServer fpfh_service;
        //ros::Subscriber points_subscriber;

    public:
        //FPFHNode(ros::NodeHandle &n)
        //FPFHNode()
        //{
        //}
        //FPFHNode() { }
        FPFHNode(ros::NodeHandle &);

        //void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
        //{
        //  // Solve all of perception here...
        //}

        void message_cb(const sensor_msgs::Image::ConstPtr& , const sensor_msgs::PointCloud2::ConstPtr& );
        //{
          // Solve all of perception here...
        void process_point_cloud(const sensor_msgs::PointCloud2::ConstPtr&, feature_extractor_fpfh::FPFHHist &);

        bool fpfh_srv_cb(feature_extractor_fpfh::FPFHCalc::Request &, feature_extractor_fpfh::FPFHCalc::Response &);
        //}
};

//class FPFHNode                                                                                         
//{       
//    protected:
//        ros::NodeHandle n_;
//        ros::Publisher hist_publisher;
//        ros::Subscriber points_subscriber;
//
//    public:
//        FPFHNode(ros::NodeHandle &n);
//        //void points_cb(const sensor_msgs::PointCloud2::ConstPtr &input_cloud, const sensor_msgs::Image::ConstPtr &image);
//        //void ex_points_cb(int a, int b);
//
//
//}; 
#endif  //#ifndef FPFHNODE_

