#ifndef HRL_OBJECT_FETCHING_TABLETOP_APPROACH_H
#define HRL_OBJECT_FETCHING_TABLETOP_APPROACH_H
#include <numeric>
#include <ros/ros.h>
#include "tabletop_object_detector/TabletopSegmentation.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include "pcl/point_types.h"
#include <boost/make_shared.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>

using namespace sensor_msgs;
using namespace tabletop_object_detector;
using namespace std;
namespace enc = sensor_msgs::image_encodings;


namespace hrl_object_fetching {
    class TabletopApproach {
        public:
            TabletopApproach();
            ros::ServiceClient tab_seg_client;
            ros::Subscriber pc_sub;
            ros::NodeHandle nh;
            ros::Publisher pc_pub;
            image_transport::ImageTransport img_trans;
            image_transport::Publisher height_pub;
            tf::TransformListener tf_listener;
            void onInit();
            bool tabletopSegCallback(TabletopSegmentation::Request& req, TabletopSegmentation::Response& resp);
            void pcCallback(PointCloud2::ConstPtr pc);
    };
};
#endif //HRL_OBJECT_FETCHING_TABLETOP_APPROACH_H
