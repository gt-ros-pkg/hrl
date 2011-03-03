#ifndef FPFHNODE_
#define FPFHNODE_
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"                                                                   
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

class FPFHNode
{
    protected:
        ros::NodeHandle n_;
        ros::Publisher hist_publisher;
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

        void callback(const sensor_msgs::Image::ConstPtr& , const sensor_msgs::PointCloud2::ConstPtr& );
        //{
          // Solve all of perception here...
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

