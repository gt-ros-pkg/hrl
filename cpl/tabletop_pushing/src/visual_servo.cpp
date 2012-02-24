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
#include <pcl_ros/transforms.h>

// Boost
#include <boost/shared_ptr.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// STL
#include <vector>
#include <deque>
#include <queue>
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

#define DEBUG_MODE 1

typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy;
using boost::shared_ptr;

class VisualServoNode
{
public:
    VisualServoNode(ros::NodeHandle &n) :
    n_(n), n_private_("~"),
    image_sub_(n, "color_image_topic", 1),
    depth_sub_(n, "depth_image_topic", 1),
    cloud_sub_(n, "point_cloud_topic", 1),
    sync_(MySyncPolicy(15), image_sub_, depth_sub_, cloud_sub_),
    it_(n), tf_(), have_depth_data_(false) 
    {
    	tf_ = shared_ptr<tf::TransformListener>(new tf::TransformListener());
        // Legacy stuff. Must remove unused ones
        n_private_.param("display_wait_ms", display_wait_ms_, 3);
        std::string default_workspace_frame = "/torso_lift_link";
        n_private_.param("workspace_frame", workspace_frame_, default_workspace_frame);
        n_private_.param("num_downsamples", num_downsamples_, 2);
        std::string cam_info_topic_def = "/kinect_head/rgb/camera_info";
        n_private_.param("cam_info_topic", cam_info_topic_, cam_info_topic_def);
        
        // color segmentation specific ones
        n_private_.param("tape_hue_value", tape_hue_value_, 137);
        n_private_.param("tape_hue_threshold", tape_hue_threshold_, 10);
        n_private_.param("hand_hue_value", hand_hue_value_, 173);
        n_private_.param("hand_hue_threshold", hand_hue_threshold_, 10);
        n_private_.param("default_sat_bot_value", default_sat_bot_value_, 40);
        n_private_.param("default_sat_top_value", default_sat_top_value_, 40);
        n_private_.param("default_val_value", default_val_value_, 200);
        n_private_.param("min_contour_size", min_contour_size_, 100.0);
        // Setup ros node connections
        sync_.registerCallback(&VisualServoNode::sensorCallback, this);
    }
    
    /** 
    * Experiment: visual servo
    * Given a fixed engineered setting, we are taping robot hand with
    * three painter's tape and use those three features to do the image-based
    * visual servoing
    **/
    void sensorCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        if (!camera_initialized_)
        {
            cam_info_ = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic_, n_, ros::Duration(2.0));
            camera_initialized_ = true;
        }
        // Convert images to OpenCV format
        cv::Mat color_frame(bridge_.imgMsgToCv(img_msg));
        cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));
        
        // Swap kinect color channel order
        cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);
	    
        // Downsample the image
        //cv::Mat color_frame_down = downSample(color_frame, num_downsamples_);
        //cv::Mat depth_frame_down = downSample(depth_frame, num_downsamples_);
	
	    // Segment the blue tape on the hand
	    // Values are from the launch file
        cv::Mat tape_mask = colorSegment(color_frame.clone(), tape_hue_value_, tape_hue_threshold_);
       
        std::vector<cv::Moments> ms = findMoments(tape_mask, color_frame); 
        ms = orderMoments(ms);

        XYZPointCloud cloud; 
        pcl::fromROSMsg(*cloud_msg, cloud);
        tf_->waitForTransform(workspace_frame_, cloud.header.frame_id,
                        cloud.header.stamp, ros::Duration(0.5));
        pcl_ros::transformPointCloud(workspace_frame_, cloud, cloud, *tf_);

        // interaction matrix, image jacobian
        double L[6][6];
        if (ms.size() == 3) {
        printf("Image Jacobian\n");
           for (int i = 0; i < 3; i++) {
               cv::Moments m = ms.at(i);
               double x = (m.m10/m.m00);
               double y = (m.m01/m.m00);
               double z = depth_frame.at<uchar>((int)y,(int)x); 
  
	       pcl::PointXYZ cur_pt = cloud.at(x, y);
               printf("[%.3f vs. %.3f]\t", z, cur_pt.z);
	       int l = i * 2;
               if (z == 0) z = 1e-4;
               L[l][0] = -1/z;
               L[l+1][0] = 0;
               L[l][1] = 0;
               L[l+1][1] = -1/z;
               L[l][2] = x/z;
               L[l+1][2] = y/z;
               L[l][3] = x*y;
               L[l+1][3] = 1 + pow(y,2);
               L[l][4] = -(1+pow(x,2));
               L[l+1][4] = -x*y;
               L[l][5] = y;
               L[l+1][5] = -x;
           }
           printf("\n");

           for (int i=0; i < 6; i++) {
               for (int j=0; j< 6; j++)
                   printf("%6.3e\t", L[i][j]);
               printf("\n");
           }
        }
        
#ifdef DEBUG_MODE
        for (unsigned int i = 0; i < ms.size(); i++) {
            cv::Moments m = ms.at(i);
            double x = m.m10/m.m00;
            double y = m.m01/m.m00;
            cv::circle(color_frame, cv::Point(x,y), 2, cv::Scalar(100*i, 0, 110*(2-i)), 2);
       } 
        
        double depth_max = 1.0;
        cv::minMaxLoc(depth_frame, NULL, &depth_max);
        cv::Mat depth_display = depth_frame.clone();
        depth_display /= depth_max;
        cv::imshow("input_depth", depth_display);
        cv::imshow("input", color_frame.clone());
        cv::waitKey(display_wait_ms_);
#endif 
    }    

    
    
    std::vector<cv::Moments> orderMoments(std::vector<cv::Moments> ms)
    {
        std::vector<cv::Moments> oMs;
        oMs.clear();
        if (ms.size() == 3) { 
            double centroids[3][2];
            for (int i = 0; i < 3; i++) {
                cv::Moments m0 = ms.at(i);
                double x0, y0;
                x0 = m0.m10/m0.m00;
                y0 = m0.m01/m0.m00;
                centroids[i][0] = x0; 
                centroids[i][1] = y0; 
            }
        
            // find the top left corner using distance scheme
            double dist[3][3], lowest(1e6);
            int one(-1);
            for (int i = 0; i < 3; i++) {
                for (int j = i; j < 3; j++) {
                    double temp = pow(centroids[j][0]- centroids[i][0],2) 
                    + pow(centroids[j][1]-centroids[i][1],2);
                    dist[i][j] = temp;
                    dist[j][i] = temp;
                }
                double score = dist[i][0] + dist[i][1] + dist[i][2];
                if (score < lowest) {
                    lowest = score;
                    one = i;
                }
            }

            oMs.push_back(ms.at(one));
            // going to use quaderant based searching
            int a = one == 0 ? 1 : 0;
            int b = one == 2 ? 1 : 2; 
            double vX0, vY0, vX1, vY1, result;
            vX0 = centroids[a][0] - centroids[one][0];
            vY0 = centroids[a][1] - centroids[one][1];
            vX1 = centroids[b][0] - centroids[one][0];
            vY1 = centroids[b][1] - centroids[one][1];
            // cross-product assuming that z = 0 for both
            result = vX1*vY0 - vX0*vY1;
            if (result >= 0) {
                oMs.push_back(ms.at(b));
                oMs.push_back(ms.at(a));
            }
            else {
                oMs.push_back(ms.at(a));
                oMs.push_back(ms.at(b));
            }
                
        }
        return oMs;
    } 

    /**
    * findMoments those morphology to filter out noises and find contours around
    * possible features. Lastly, it returns the three largest moments
    * Arg
    * in: single channel image input
    * Return
    * three largest moment in the image
    */
    std::vector<cv::Moments> findMoments(cv::Mat in, cv::Mat &color_frame) { 
        cv::Mat open, temp;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
        cv::morphologyEx(in.clone(), open, cv::MORPH_OPEN, element);
	std::vector<std::vector<cv::Point> > contours; contours.clear();
	temp = open.clone();
    	cv::findContours(temp, contours, cv::RETR_CCOMP,CV_CHAIN_APPROX_NONE);
        std::vector<cv::Moments> moments; moments.clear();
        
        for (unsigned int i = 0; i < contours.size(); i++) {
            cv::Moments m = cv::moments(contours[i]);
            if (m.m00 > min_contour_size_) {
                // first add the forth element

                moments.push_back(m);
                // find the smallest element of 4 and remove that
                if (moments.size() > 3) {
                    double small(moments.at(0).m00);
                    unsigned int smallInd(0);
                    for (unsigned int j = 1; j < moments.size(); j++){
                        if (moments.at(j).m00 < small) {
                            small = moments.at(j).m00;
                            smallInd = j;
                        }
                    }
                    moments.erase(moments.begin() + smallInd);
                }
            }
        }

#ifdef DEBUG_MODE
        cv::drawContours(color_frame, contours, -1,  cv::Scalar(50,225,255), 2);
        cv::imshow("open", open.clone());   
#endif
 

        return moments;
    }
    
    cv::Mat colorSegment(cv::Mat color_frame, int hue, int threshold){
        /*
         * Often value = 0 or 255 are very useless. 
         * The distance at those end points get very close and it is not useful
         * Same with saturation 0. Low saturation makes everything more gray scaled
         * So the default setting are below 
         */
        return colorSegment(color_frame, hue - threshold, hue + threshold,  default_sat_bot_value_, default_sat_top_value_, 50, default_val_value_);
    }

    
    /** 
     * colorSegment
     * Very Basic Color Segmentation done in HSV space
     * Takes in Hue value and threshold as input to compute the distance in color space
     * cv::Mat color_frame: color input from image
     * Return: mask from the color segmentation 
     */
    cv::Mat colorSegment(cv::Mat color_frame, int _hue_n, int _hue_p, int _sat_n, int _sat_p, int _value_n,  int _value_p){
        cv::Mat temp (color_frame.clone());
        cv::cvtColor(temp, temp, CV_RGB2HSV);
        std::vector<cv::Mat> hsv;
        cv::split(temp, hsv);
        
        // masking out values that do not fall between the condition 
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

        // removing unwanted parts by applying mask to the original image
        return wm;
    }
   
    
    /*************************
        Helper Methods
    **************************/
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
    shared_ptr<tf::TransformListener> tf_;
    cv::Mat cur_color_frame_;
    cv::Mat cur_depth_frame_;
    cv::Mat cur_workspace_mask_;
    cv::Mat prev_color_frame_;
    cv::Mat prev_depth_frame_;
    cv::Mat prev_workspace_mask_;
    std_msgs::Header cur_camera_header_;
    std_msgs::Header prev_camera_header_;
    XYZPointCloud cur_point_cloud_;
    bool have_depth_data_;
    int display_wait_ms_;
    int num_downsamples_;
    std::string workspace_frame_;
    bool camera_initialized_;
    std::string cam_info_topic_;
    int tracker_count_;
    int tape_hue_value_;
    int tape_hue_threshold_;
    int hand_hue_value_;
    int hand_hue_threshold_;
    int default_sat_bot_value_;
    int default_sat_top_value_;
    int default_val_value_;
    double min_contour_size_;
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
