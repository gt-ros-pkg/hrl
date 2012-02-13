/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Georgia Institute of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/norms.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/registration/icp.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost
// TODO: Use these instead of passing pointers about
#include <boost/shared_ptr.hpp>

// cpl_visual_features
#include <cpl_visual_features/motion/flow_types.h>
#include <cpl_visual_features/motion/dense_lk.h>
#include <cpl_visual_features/motion/feature_tracker.h>

// tabletop_pushing
#include <tabletop_pushing/PushPose.h>
#include <tabletop_pushing/LocateTable.h>

// STL
#include <vector>
#include <deque>
#include <queue>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <utility>
#include <stdexcept>
#include <float.h>
#include <math.h>
#include <time.h> // for srand(time(NULL))
#include <cstdlib> // for MAX_RAND

// Debugging IFDEFS
#define DISPLAY_INPUT_COLOR 1
#define DISPLAY_INPUT_DEPTH 1
// #define DISPLAY_WORKSPACE_MASK 1
// #define DISPLAY_PLANE_ESTIMATE 1
// #define DISPLAY_TABLE_DISTANCES 1
#define DISPLAY_OBJECT_BOUNDARIES 1
#define DISPLAY_PROJECTED_OBJECTS 1
#define DISPLAY_OBJECT_SPLITS 1
// #define DISPLAY_LINKED_EDGES 1
// #define DISPLAY_CHOSEN_BOUNDARY 1
#define DISPLAY_CLOUD_DIFF 1
#define DISPLAY_3D_BOUNDARIES 1
#define DISPLAY_WAIT 1

#define randf() static_cast<float>(rand())/RAND_MAX


/
class VisualServoNode
{
public:
    VisualServoNode(ros::NodeHandle &n) :
    n_(n), n_private_("~"),
    image_sub_(n, "color_image_topic", 1),
    depth_sub_(n, "depth_image_topic", 1),
    cloud_sub_(n, "point_cloud_topic", 1),
    sync_(MySyncPolicy(15), image_sub_, depth_sub_, cloud_sub_),
    it_(n), tf_(), ft_("pushing_perception"),
    pcl_segmenter_(&ft_, &tf_),
    
    have_depth_data_(false), tracking_(false),
    tracker_initialized_(false), tracker_count_(0)
    {
        // Get parameters from the server
        n_private_.param("crop_min_x", crop_min_x_, 0);
        n_private_.param("crop_max_x", crop_max_x_, 640);
        n_private_.param("crop_min_y", crop_min_y_, 0);
        n_private_.param("crop_max_y", crop_max_y_, 480);
        n_private_.param("display_wait_ms", display_wait_ms_, 3);
        n_private_.param("min_workspace_x", min_workspace_x_, 0.0);
        n_private_.param("min_workspace_y", min_workspace_y_, 0.0);
        n_private_.param("min_workspace_z", min_workspace_z_, 0.0);
        n_private_.param("max_workspace_x", max_workspace_x_, 0.0);
        n_private_.param("max_workspace_y", max_workspace_y_, 0.0);
        n_private_.param("max_workspace_z", max_workspace_z_, 0.0);
        std::string default_workspace_frame = "/torso_lift_link";
        n_private_.param("workspace_frame", workspace_frame_, default_workspace_frame);
        
        n_private_.param("min_table_z", pcl_segmenter_.min_table_z_, -0.5);
        n_private_.param("max_table_z", pcl_segmenter_.max_table_z_, 1.5);
        pcl_segmenter_.min_workspace_x_ = min_workspace_x_;
        pcl_segmenter_.max_workspace_x_ = max_workspace_x_;
        pcl_segmenter_.min_workspace_z_ = min_workspace_z_;
        pcl_segmenter_.max_workspace_z_ = max_workspace_z_;
        
        n_private_.param("autostart_tracking", tracking_, false);
        n_private_.param("autostart_pcl_segmentation", autorun_pcl_segmentation_, false);
        n_private_.param("use_guided_pushes", use_guided_pushes_, true);
        
        n_private_.param("num_downsamples", num_downsamples_, 2);
        pcl_segmenter_.num_downsamples_ = num_downsamples_;
        std::string cam_info_topic_def = "/kinect_head/rgb/camera_info";
        n_private_.param("cam_info_topic", cam_info_topic_, cam_info_topic_def);
        n_private_.param("table_ransac_thresh", pcl_segmenter_.table_ransac_thresh_, 0.01);
        n_private_.param("table_ransac_angle_thresh", pcl_segmenter_.table_ransac_angle_thresh_, 30.0);
        
        n_private_.param("surf_hessian_thresh", ft_.surf_.hessianThreshold, 150.0);
        bool use_fast;
        n_private_.param("use_fast_corners", use_fast, false);
        ft_.setUseFast(use_fast);
        n_private_.param("pcl_cluster_tolerance", pcl_segmenter_.cluster_tolerance_, 0.25);
        n_private_.param("pcl_difference_thresh", pcl_segmenter_.cloud_diff_thresh_, 0.01);
        n_private_.param("pcl_min_cluster_size", pcl_segmenter_.min_cluster_size_, 100);
        n_private_.param("pcl_max_cluster_size", pcl_segmenter_.max_cluster_size_, 2500);
        n_private_.param("pcl_voxel_downsample_res", pcl_segmenter_.voxel_down_res_, 0.005);
        n_private_.param("pcl_cloud_intersect_thresh", pcl_segmenter_.cloud_intersect_thresh_, 0.005);
        n_private_.param("pcl_concave_hull_alpha", pcl_segmenter_.hull_alpha_, 0.1);
        n_private_.param("use_pcl_voxel_downsample", pcl_segmenter_.use_voxel_down_, true);
        
        n_private_.param("tape_hue_value", tape_hue_value_, 137);
        n_private_.param("tape_hue_threshold", tape_hue_threshold_, 10);
        n_private_.param("hand_hue_value", hand_hue_value_, 173);
        n_private_.param("hand_hue_threshold", hand_hue_threshold_, 10);
        n_private_.param("default_push_dist", default_push_dist_, 0.1);
        // Setup ros node connections
        sync_.registerCallback(&VisualServoNode::sensorCallback,       this);
    }
    
    
    void sensorCallback(const sensor_msgs::ImageConstPtr& img_msg,const sensor_msgs::ImageConstPtr& depth_msg,const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        if (!camera_initialized_)
        {
            cam_info_ = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic_, n_, ros::Duration(5.0));
            camera_initialized_ = true;
            pcl_segmenter_.cam_info_ = cam_info_;
        }
        // Convert images to OpenCV format
        cv::Mat color_frame(bridge_.imgMsgToCv(img_msg));
        cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));
        
        // Swap kinect color channel order
        cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);
        
        
        /** 
         * Uncomment below to test with local image
         **/
        //IplImage* img = cvLoadImage("/home/bootcamp/gt-ros-pkg/cpl/tabletop_pushing/src/test1.jpg");
        //cv::Mat readin(img);
        //cvReleaseImage(&img);
        //cv::imshow("input",readin.clone());
        //getBlueTape(readin.clone());
        cv::Mat color_frame_down = downSample(color_frame, num_downsamples_);
        cv::Mat depth_frame_down = downSample(depth_frame, num_downsamples_);
        
        cv::Mat tape = colorSegment(color_frame_down.clone(), tape_hue_value_, tape_hue_threshold_);
        cv::Mat hand = colorSegment(color_frame_down.clone(), hand_hue_value_, hand_hue_threshold_);
        cv::imshow("input",color_frame_down.clone());
        cv::imshow("tape", tape.clone()); 
        cv::imshow("hand", hand.clone());   
        cv::waitKey(display_wait_ms_);
    }
    
    
    
    cv::Mat colorSegment(cv::Mat color_frame, int hue, int threshold){
        /*
         * Often value = 0 or 255 are very useless. The distance at those end points get very close and it is not useful
         * Same with saturation 0. Low saturation makes everything more gray scaled
         * So the default setting are below 
         */
        return colorSegment(color_frame, hue - threshold, hue + threshold,  40, 255, 30, 235);
    }

    
    /** 
     * colorSegment
     * Very Basic Color Segmentation done in HSV space
     * Takes in Hue value and threshold as input to compute the distance in color space
     * cv::Mat color_frame: color input from image
     * 
     */
    cv::Mat colorSegment(cv::Mat color_frame, int _hue_n, int _hue_p, int _sat_n, int _sat_p, int _value_n,  int _value_p){
        cv::Mat temp (color_frame.clone());
        cv::cvtColor(temp, temp, CV_RGB2HSV);
        std::vector<cv::Mat> hsv;
        cv::split(temp, hsv);
        
         //std::cout <<"[" << (int)hsv[0].at<uchar>(0, 0) << "]";
         //std::cout <<"[" << (int)hsv[1].at<uchar>(0, 0) << "]";
         //std::cout <<"[" << (int)hsv[2].at<uchar>(0, 0) << "]\n";
        cv::Mat wm(color_frame.rows, color_frame.cols, CV_8UC1, cv::Scalar(0));
        for (int r = 0; r < temp.rows; r++)
        {
            uchar* workspace_row = wm.ptr<uchar>(r);
            for (int c = 0; c < temp.cols; c++)
            {
                int hue = (int)hsv[0].at<uchar>(r, c), sat = (int)hsv[1].at<uchar>(r, c), value = (int)hsv[2].at<uchar>(r, c);
                if (_hue_n < hue && hue < _hue_p)
                    if (_sat_n < sat && sat < _sat_p)
                        if (_value_n < value && value < _value_p)
                            workspace_row[c] = 255;
                
            } 
        }
        cv::Mat ret;
        color_frame.copyTo(ret, wm);
        return ret;
    }
    
    
    
    
    //
    // Helper Methods
    //
    cv::Mat downSample(cv::Mat data_in, int scales)
    {
        cv::Mat out = data_in.clone();
        for (int i = 0; i < scales; ++i)
        {
            cv::pyrDown(data_in, out);
            data_in = out;
        }
        return out;
    }
    
    cv::Mat upSample(cv::Mat data_in, int scales)
    {
        cv::Mat out = data_in.clone();
        for (int i = 0; i < scales; ++i)
        {
            // NOTE: Currently assumes even cols, rows for data_in
            cv::Size out_size(data_in.cols*2, data_in.rows*2);
            cv::pyrUp(data_in, out, out_size);
            data_in = out;
        }
        return out;
    }
    
    /**
     * Executive control function for launching the node.
     */
    void spin()
    {
        while(n_.ok())
        {
            ros::spinOnce();
        }
    }
    
protected:
    ros::NodeHandle n_;
    ros::NodeHandle n_private_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Synchronizer<MySyncPolicy> sync_;
    image_transport::ImageTransport it_;
    sensor_msgs::CameraInfo cam_info_;
    sensor_msgs::CvBridge bridge_;
    tf::TransformListener tf_;
    cv::Mat cur_color_frame_;
    cv::Mat cur_depth_frame_;
    cv::Mat cur_workspace_mask_;
    cv::Mat prev_color_frame_;
    cv::Mat prev_depth_frame_;
    cv::Mat prev_workspace_mask_;
    std_msgs::Header cur_camera_header_;
    std_msgs::Header prev_camera_header_;
    XYZPointCloud cur_point_cloud_;
    PointCloudSegmentation pcl_segmenter_;
    bool have_depth_data_;
    int crop_min_x_;
    int crop_max_x_;
    int crop_min_y_;
    int crop_max_y_;
    int display_wait_ms_;
    double min_workspace_x_;
    double max_workspace_x_;
    double min_workspace_y_;
    double max_workspace_y_;
    double min_workspace_z_;
    double max_workspace_z_;
    int num_downsamples_;
    std::string workspace_frame_;
    bool tracking_;
    bool tracker_initialized_;
    bool camera_initialized_;
    std::string cam_info_topic_;
    int tracker_count_;
    bool autorun_pcl_segmentation_;
    bool use_guided_pushes_;
    double default_push_dist_;
    int tape_hue_value_;
    int tape_hue_threshold_;
    int hand_hue_value_;
    int hand_hue_threshold_;
};

int main(int argc, char ** argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "visual_servo_node");
    ros::NodeHandle n;
    VisualServoNode singulation_node(n);
    singulation_node.spin();
    return 0;
}
