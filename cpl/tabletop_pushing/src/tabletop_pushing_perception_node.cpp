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

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Visual features
#include <cpl_visual_features/sliding_window.h>
#include <cpl_visual_features/features/hsv_color_histogram.h>
#include <cpl_visual_features/features/attribute_learning_base_feature.h>

// Superpixels
#include <cpl_superpixels/segment/segment.h>

// tabletop_pushing
#include <tabletop_pushing/PushPose.h>

// STL
#include <vector>
#include <queue>
#include <string>
#include <sstream>
#include <fstream>
#include <utility>

#define CALL_PUSH_POSE_ON_CALLBCK

using cpl_superpixels::getSuperpixelImage;
using tabletop_pushing::PushPose;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image> MySyncPolicy;
// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
//                                                         sensor_msgs::Image,
//                                                         sensor_msgs::PointCloud2> MySyncPolicy;

class TabletopPushingPerceptionNode
{
 public:
  TabletopPushingPerceptionNode(ros::NodeHandle &n) :
      n_(n),
      image_sub_(n, "color_image_topic", 1),
      depth_sub_(n, "depth_image_topic", 1),
      // point_sub_(n, "point_cloud_topic", 1),
      // sync_(MySyncPolicy(1), image_sub_, depth_sub_, point_sub_),
      sync_(MySyncPolicy(1), image_sub_, depth_sub_),
      tf_(), have_depth_data_(false)
  {
    ros::NodeHandle n_private("~");
    n_private.param("segment_k", k_, 500.0);
    n_private.param("segment_sigma", sigma_, 0.9);
    n_private.param("segment_min_size", min_size_, 30);
    n_private.param("segment_color_weight", wc_, 0.1);
    n_private.param("segment_depth_weight", wd_, 0.7);
    sync_.registerCallback(&TabletopPushingPerceptionNode::sensorCallback,
                           this);
    push_pose_server_ = n_.advertiseService(
        "get_push_pose", &TabletopPushingPerceptionNode::getPushPose, this);
  }

  // TODO: Should we change this to actively poll the camera when the service
  // is called?
  void sensorCallback(const sensor_msgs::ImageConstPtr& img_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg)
  {
    // Convert images to OpenCV format
    cv::Mat visual_frame(bridge_.imgMsgToCv(img_msg));
    cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));
    // XYZPointCloud cloud;
    // pcl::fromROSMsg(*cloud_msg, cloud);
    // Save internally for use in the service callback
    cur_visual_frame_ = visual_frame;
    cur_depth_frame_ = depth_frame;
    // cur_point_cloud_ = cloud;
    have_depth_data_ = true;
#ifdef CALL_PUSH_POSE_ON_CALLBCK
    PushPose::Response p = findPushPose(cur_visual_frame_, cur_depth_frame_,
                                        cur_point_cloud_);
#endif // CALL_PUSH_POSE_ON_CALLBCK
  }

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    if ( have_depth_data_ )
    {
      res = findPushPose(cur_visual_frame_, cur_depth_frame_, cur_point_cloud_);
    }
    else
    {
      ROS_ERROR_STREAM("Calling getPushPose prior to receiving sensor data.");
      return false;
    }
    return true;
  }

  PushPose::Response findPushPose(cv::Mat& visual_frame,
                                  cv::Mat& depth_frame,
                                  XYZPointCloud& points)
  {
    return superpixelFindPushPose(visual_frame, depth_frame, points);
  }

  PushPose::Response superpixelFindPushPose(cv::Mat& visual_frame,
                                            cv::Mat& depth_frame,
                                            XYZPointCloud& points)
  {
    cv::cvtColor(visual_frame, visual_frame, CV_RGB2BGR);

    // TODO: Fix normal estimation
    // pcl::IntegralImageNormalEstimation ne;
    // pcl::PointCloud<pcl::Normal> normals;
    // ne.compute(points, normals, 0.02f, 10.0f);//, pcl::AVERAGE_DEPTH_CHANGE);

    // TODO: Fill in gaps inside the depth data.
    // TODO: Select inner ROI of images to remove border issues?
    int num_regions = 0;
    int num_vis_regions = 0;
    cv::Mat regions = getSuperpixelImage(visual_frame, depth_frame, num_regions,
                                         sigma_, k_, min_size_, wc_, wd_);
    cv::Mat visual_regions = getSuperpixelImage(visual_frame, num_vis_regions,
                                                sigma_, k_, min_size_);
    cv::imshow("visual_frame", visual_frame);
    cv::imshow("depth_frame", depth_frame);
    cv::imshow("vis_regions", visual_regions);
    cv::imshow("vis_depth_regions", regions);

    ROS_INFO_STREAM("Computed " << num_regions << " regions");
    ROS_INFO_STREAM("Computed " << num_vis_regions << " visual regions");
    cv::waitKey();

    // TODO: Choose a patch based on some simple criterian
    // TODO: Estimate the surface of the patch from the depth image
    // TODO: Extract the push pose as point in the center of that surface
    // TODO: Transform to be in the torso_lift_link
    PushPose::Response res;
    geometry_msgs::PoseStamped p;
    res.push_pose = p;
    res.invalid_push_pose = false;
    return res;
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
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  // message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub_;
  message_filters::Synchronizer<MySyncPolicy> sync_;
  sensor_msgs::CvBridge bridge_;
  tf::TransformListener tf_;
  ros::ServiceServer push_pose_server_;
  cv::Mat cur_visual_frame_;
  cv::Mat cur_depth_frame_;
  XYZPointCloud cur_point_cloud_;
  bool have_depth_data_;
  double k_;
  double sigma_;
  int min_size_;
  double wc_;
  double wd_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  TabletopPushingPerceptionNode perception_node(n);
  perception_node.spin();
}
