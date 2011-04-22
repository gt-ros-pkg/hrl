#ifndef HRL_TABLE_DETECTION_TABLETOP_DETECTOR_H
#define HRL_TABLE_DETECTION_TABLETOP_DETECTOR_H
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
#include <hrl_table_detect/DetectTableStart.h>
#include <hrl_table_detect/DetectTableStop.h>
#include <hrl_table_detect/DetectTableInst.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <hrl_cvblobslib/Blob.h>
#include <hrl_cvblobslib/BlobResult.h>

using namespace sensor_msgs;
using namespace std;
namespace enc = sensor_msgs::image_encodings;


namespace hrl_table_detect {
    class TabletopDetector {
        public:
            TabletopDetector();
            // Parameters
            double minx, maxx, miny, maxy, minz, maxz, resolution, imgx, imgy;
            double inlier_magnitude; 
            int32_t num_edge_dilate, num_closes;
            double degree_bins, hough_thresh;
            double theta_gran, rho_gran;
            double xgran, ygran;

            ros::ServiceServer table_detect_start;
            ros::ServiceServer table_detect_stop;
            ros::ServiceServer table_detect_inst;
            ros::Subscriber pc_sub;
            ros::NodeHandle nh;
            ros::NodeHandle nh_priv;
            ros::Publisher pc_pub;
            ros::Publisher pose_arr_pub;
            cv::Mat height_img_sum, height_img_count, height_img_max;
            bool grasp_points_found;
            geometry_msgs::PoseArray grasp_points;
            image_transport::ImageTransport img_trans;
            image_transport::Publisher height_pub;
            tf::TransformListener tf_listener;
            boost::mutex pc_lock;
            void onInit();
            void pcCallback(PointCloud2::ConstPtr pc);
            bool startCallback(hrl_table_detect::DetectTableStart::Request& req, 
                             hrl_table_detect::DetectTableStart::Response& resp);
            bool stopCallback(hrl_table_detect::DetectTableStop::Request& req, 
                             hrl_table_detect::DetectTableStop::Response& resp);
            bool instCallback(hrl_table_detect::DetectTableInst::Request& req, 
                             hrl_table_detect::DetectTableInst::Response& resp);
    };
};
#endif //HRL_OBJECT_FETCHING_TABLETOP_DETECTOR_H
