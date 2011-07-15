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
#include <sensor_msgs/Image.h>
#include <math.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/CvBridge.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Visual features
#include <cpl_visual_features/saliency/center_surround.h>
// #include <cpl_visual_features/sliding_window.h>


// STL
#include <vector>
#include <string>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;

class DepthSaliencyNode
{
 public:
  DepthSaliencyNode(ros::NodeHandle &n) :
      n_(n),
      image_sub_(n,"/kinect_head/rgb/image_color", 1),
      depth_sub_(n,"/kinect_head/depth/image", 1),
      sync_(MySyncPolicy(1), image_sub_, depth_sub_),
      csm(2,3,3,4), img_count_(0)
  {
    // Combine the subscribed topics with a message filter
    sync_.registerCallback(&DepthSaliencyNode::callback, this);
    ROS_INFO_STREAM("Synced callbacks.");
  }

  void callback(const sensor_msgs::ImageConstPtr& img_msg,
                const sensor_msgs::ImageConstPtr& depth_msg)
  {
    // Convert images to OpenCV format
    cv::Mat input_frame(bridge_.imgMsgToCv(img_msg));
    cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));
    cv::cvtColor(input_frame, input_frame, CV_RGB2BGR);
    ROS_INFO_STREAM("input_frame.type()" << input_frame.type());
    cv::imshow("input_frame", input_frame);
    cv::imshow("depth_frame", depth_frame);

    // Save frames to disk
    cv::Mat depth_to_save(input_frame.size(), CV_16UC1);
    std::vector<int> params;
    params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    params.push_back(0);
    for (int r = 0; r < depth_frame.rows; ++r)
    {
      for (int c = 0; c < depth_frame.cols; ++c)
      {
        if (isnan(depth_frame.at<float>(r,c)))
          depth_frame.at<float>(r,c) = 0.0;
      }
    }
    ROS_INFO_STREAM("Removed NaNs");

#ifdef SAVE_IMAGES
    std::stringstream intensity_name;
    intensity_name << "/home/thermans/Desktop/intensity" << img_count_
                   << ".png";
    std::stringstream depth_name;
    depth_name << "/home/thermans/Desktop/depth" << img_count_++ << ".png";
    depth_frame.convertTo(depth_to_save, depth_to_save.type(), 512.0);
    cv::imwrite(intensity_name.str(), input_frame, params);
    cv::imwrite(depth_name.str(), depth_to_save, params);
    ROS_INFO_STREAM("Saved images");
#endif // SAVE_IMAGES

    ROS_INFO_STREAM("Scaling image");
    cv::Mat scaled_depth = depth_frame.clone();
    cv::Mat scaled_depth_frame = csm.scaleMap(depth_frame);
    cv::imshow("scaled_frame", scaled_depth_frame);
    scaled_depth_frame.convertTo(scaled_depth, CV_32FC1);
    ROS_INFO_STREAM("Running saliency");
    cv::Mat saliency_map = csm(input_frame, scaled_depth);
    // cv::Mat saliency_map = csm(input_frame, depth_frame);
    double min_val = 0;
    double max_val = 0;
    cv::minMaxLoc(saliency_map, &min_val, &max_val);
    ROS_INFO_STREAM("Minimum saliency unscaled: " << min_val << "\tmax: "
                    << max_val);
    cv::imshow("unsaceled saliency", saliency_map);
    cv::waitKey();
  }

  cv::Mat upSampleResponse(cv::Mat& m_s, int s, cv::Size size0)
  {
    // Upsample from scale s to scale 0
    cv::Mat m_s_prime;
    cv::Mat temp;
    m_s.copyTo(m_s_prime);
    m_s.copyTo(temp);

    // Calculate the correct sizes to up sample to
    std::vector<cv::Size> sizes;
    cv::Size current_size = size0;
    for (int i = 0; i < s; i++)
    {
      sizes.push_back(current_size);
      current_size.width /= 2;
      current_size.height /= 2;
    }

    for (int i = 0; i < s; i++)
    {
      cv::Size up_size;
      up_size = sizes.back();
      sizes.pop_back();
      cv::pyrUp(temp, m_s_prime, up_size);
      temp = m_s_prime;
    }

    return m_s_prime;
  }

  /**
   * Executive control function for launching the node.
   */
  void spin()
  {
    ROS_INFO_STREAM("Spinning!");
    while(n_.ok())
    {
      ros::spinOnce();
    }
  }

 protected:
  ros::NodeHandle n_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  // message_filters::TimeSynchronizer<sensor_msgs::Image,
  //                                   sensor_msgs::Image> sync_;
  message_filters::Synchronizer<MySyncPolicy> sync_;
  cpl_visual_features::CenterSurroundMapper csm;
  sensor_msgs::CvBridge bridge_;
  int img_count_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "depth_saliency_node");
  ros::NodeHandle n;
  DepthSaliencyNode saliency_node(n);
  saliency_node.spin();
}
