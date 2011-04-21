#ifndef HRL_OBJECT_FETCHING_TABLETOP_DETECTOR_H
#define HRL_OBJECT_FETCHING_TABLETOP_DETECTOR_H
#include <numeric>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include "pcl/point_types.h"
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <hrl_object_fetching/DetectTable.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

using namespace sensor_msgs;
using namespace std;
namespace enc = sensor_msgs::image_encodings;


namespace hrl_object_fetching {
    class TabletopDetector {
        public:
            TabletopDetector();
            // Parameters
            double minx, maxx, miny, maxy, minz, maxz, resolution, imgx, imgy;
            double inlier_magnitude, num_edge_dilate;
            double degree_bins, hough_thresh;
            double theta_gran, rho_gran;
            double xgran, ygran;

            ros::ServiceServer table_detect_service;
            ros::Subscriber pc_sub;
            ros::NodeHandle nh;
            ros::NodeHandle nh_priv;
            ros::Publisher pc_pub;
            ros::Publisher pose_arr_pub;
            cv::Mat height_img_sum, height_img_count;
            bool grasp_points_found;
            geometry_msgs::PoseArray grasp_points;
            image_transport::ImageTransport img_trans;
            image_transport::Publisher height_pub;
            tf::TransformListener tf_listener;
            boost::mutex pc_lock;
            void onInit();
            void pcCallback(PointCloud2::ConstPtr pc);
            bool srvCallback(hrl_object_fetching::DetectTable::Request& req, 
                             hrl_object_fetching::DetectTable::Response& resp);
    };
};
#endif //HRL_OBJECT_FETCHING_TABLETOP_DETECTOR_H
