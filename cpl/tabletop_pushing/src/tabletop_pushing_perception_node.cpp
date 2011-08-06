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
#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/feature.h>
#include <pcl/common/eigen.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// tabletop_pushing
#include <tabletop_pushing/PushPose.h>
#include <tabletop_pushing/LocateTable.h>
#include <tabletop_pushing/SegTrackAction.h>

// STL
#include <vector>
#include <deque>
#include <queue>
#include <string>
#include <utility>
#include <math.h>


#define DISPLAY_MOTION_PROBS 1
#define DISPLAY_INPUT_IMAGES 1
// #define DISPLAY_INTERMEDIATE_PROBS 1

using tabletop_pushing::PushPose;
using tabletop_pushing::LocateTable;
using geometry_msgs::PoseStamped;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2> MySyncPolicy;

typedef cv::Vec<float,5> Vec5f;

class ProbImageDifferencing
{
 public:
  ProbImageDifferencing(unsigned int num_hist_frames, float T_in=3,
                        float T_out=15) :
      num_hist_frames_(num_hist_frames), ONES(1.0, 1.0, 1.0, 1.0),
      T_in_(T_in), T_out_(T_out)
  {
  }

  void initialize(cv::Mat& color_frame, cv::Mat& depth_frame)
  {
    // Restart the motion history
    motion_probs_.create(color_frame.size(), CV_32FC4);
    d_motion_probs_.create(depth_frame.size(), CV_32FC1);

    // Restart the queue
    pixel_histories_.clear();
    d_histories_.clear();
    update(color_frame, depth_frame);
  }

  // std::vector<cv::Mat> update(cv::Mat& bgrd_frame)
  std::vector<cv::Mat> update(cv::Mat& color_frame, cv::Mat& depth_frame)
  {
    // Convert to correct format
    cv::Mat bgr_frame(color_frame.size(), CV_32FC3);
    color_frame.convertTo(bgr_frame, CV_32FC3, 1./255, 0);

    // Calculate probability of being the same color and depth at each pixel
    std::vector<cv::Mat> motions;
    if (pixel_histories_.size() > 1)
      motions = calculateProbabilities(bgr_frame, depth_frame);

    // Store current frame in history
    pixel_histories_.push_back(bgr_frame);
    d_histories_.push_back(depth_frame);

    // Pop the front of the queue if we have too many frames
    if (pixel_histories_.size() > num_hist_frames_)
    {
      pixel_histories_.pop_front();
      d_histories_.pop_front();
    }

    return motions;
  }

  std::vector<cv::Mat> calculateProbabilities(cv::Mat& bgr_frame,
                                              cv::Mat& depth_frame)
  {
    // Update Gaussian estimates at each pixel
    // Calculate means
    cv::Mat means(bgr_frame.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
    cv::Mat d_means(depth_frame.size(), CV_32FC1, cv::Scalar(0.0));
    for (unsigned int i = 0; i < pixel_histories_.size(); ++i)
    {
      for (int r = 0; r < means.rows; ++r)
      {
        for (int c = 0; c < means.cols; ++c)
        {
          means.at<cv::Vec3f>(r,c) += pixel_histories_[i].at<cv::Vec3f>(r,c);
          d_means.at<float>(r,c) += d_histories_[i].at<float>(r,c);
        }
      }
    }
    means /= pixel_histories_.size();
    d_means /= d_histories_.size();

    // Calculate variances
    cv::Mat vars(bgr_frame.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
    cv::Mat d_vars(depth_frame.size(), CV_32FC1, cv::Scalar(0.0));
    for (unsigned int i = 0; i < pixel_histories_.size(); ++i)
    {
      for (int r = 0; r < means.rows; ++r)
      {
        for (int c = 0; c < means.cols; ++c)
        {
          cv::Vec3f diff = pixel_histories_[i].at<cv::Vec3f>(r,c) -
              means.at<cv::Vec3f>(r,c);

          vars.at<cv::Vec3f>(r,c) += diff.mul(diff);

          float d_diff = d_histories_[i].at<float>(r,c)-d_means.at<float>(r,c);
          d_vars.at<float>(r,c) += d_diff * d_diff;
        }
      }
    }
    vars /= (pixel_histories_.size()-1.0);
    d_vars /= (d_histories_.size()-1.0);

    int d_probs_greater = 0;
    int d_probs_lesser = 0;
    // Calculate probability of pixels having moved
    for (int r = 0; r < motion_probs_.rows; ++r)
    {
      for (int c = 0; c < motion_probs_.cols; ++c)
      {
        cv::Vec3f x = bgr_frame.at<cv::Vec3f>(r,c);
        cv::Vec3f mu = means.at<cv::Vec3f>(r,c);
        cv::Vec3f var = vars.at<cv::Vec3f>(r,c);
        // NOTE: Probability of not belonging to the gaussian
        cv::Vec4f probs = ONES - p_x_gaussian(x, mu, var);
        motion_probs_.at<cv::Vec4f>(r,c) = probs;
        float x_d = depth_frame.at<float>(r,c);
        float mu_d = d_means.at<float>(r,c);
        float var_d = d_vars.at<float>(r,c);
        float prob_d = 1.0 - p_x_gaussian(x_d, mu_d, var_d);
        if (prob_d > 1.0)
        {
          d_probs_greater++;
        }
        else if (prob_d < 0.0)
        {
          d_probs_lesser++;
          // ROS_INFO_STREAM("prob_d is: " << prob_d);
        }
        d_motion_probs_.at<float>(r,c) = prob_d;
        motion_probs_.at<cv::Vec4f>(r,c)[3]*= prob_d;
      }
    }
    // ROS_INFO_STREAM("d_probs_greater " << d_probs_greater);
    // ROS_INFO_STREAM("d_probs_lesser " << d_probs_lesser);

    // TODO: Merge these into a single image?
    std::vector<cv::Mat> motions;
    motions.push_back(motion_probs_);
    motions.push_back(d_motion_probs_);
    return motions;
  }

  cv::Vec4f p_x_gaussian(cv::Vec3f x, cv::Vec3f mu, cv::Vec3f var)
  {
    cv::Vec4f p_x;
    p_x[0] = p_x_gaussian(x[0], mu[0], var[0]);
    p_x[1] = p_x_gaussian(x[1], mu[1], var[1]);
    p_x[2] = p_x_gaussian(x[2], mu[2], var[2]);
    p_x[3] = p_x[0]*p_x[1]*p_x[2];
    return p_x;
  }

  float p_x_gaussian(float x, float mu, float var)
  {
    float mean_diff = abs(x-mu);
    float sigma = sqrt(var);
    float s0 = T_in_*sigma;
    float s1 = T_out_*sigma;

    if (mean_diff <= s0)
    {
      // Don't make things impossible (i.e. depth does not change when things
      // are too close)
      return 0.99999999;
      // return 1.0;
    }
    if (mean_diff >= s1)
    {
      return 0.0;
    }
    float m = 1.0/(s0-s1);
    return m*(mean_diff-s1);
  }

  // Gets the likelihood of the gaussian with mean mu and variance var at
  // point x
  float l_x_gaussian(float x, float mu, float var)
  {
    if (var == 0.0)
    {
      // Don't make things impossible
      if (x == mu)
      {
        return 0.99999999;
        // return 1.0;
      }
      else
        return 0.0;
    }
    float mean_diff = x-mu;
    return exp(-(mean_diff*mean_diff)/(2.0*var))/(sqrt(2.0*var*M_PI));
  }

  void setNumHistFrames(unsigned int num_hist_frames)
  {
    num_hist_frames_ = num_hist_frames;
  }
  void setT_inT_out(float T_in, float T_out)
  {
    T_in_ = T_in;
    T_out_ = T_out;
  }

 protected:
  cv::Mat motion_probs_;
  cv::Mat d_motion_probs_;
  std::deque<cv::Mat> pixel_histories_;
  std::deque<cv::Mat> d_histories_;
  unsigned int num_hist_frames_;
  const cv::Vec4f ONES;
  float T_in_;
  float T_out_;

};

class TabletopPushingPerceptionNode
{
 public:
  TabletopPushingPerceptionNode(ros::NodeHandle &n) :
      n_(n),
      image_sub_(n, "color_image_topic", 1),
      depth_sub_(n, "depth_image_topic", 1),
      cloud_sub_(n, "point_cloud_topic", 1),
      sync_(MySyncPolicy(1), image_sub_, depth_sub_, cloud_sub_),
      track_server_(n, "seg_track_action",
                    boost::bind(&TabletopPushingPerceptionNode::startTracker,
                                this, _1),
                    false),
      tf_(), motion_probs_(5), have_depth_data_(false), tracking_(true),
      tracker_initialized_(false)
  {
    // Get parameters from the server
    ros::NodeHandle n_private("~");
    n_private.param("crop_min_x", crop_min_x_, 0);
    n_private.param("crop_max_x", crop_max_x_, 640);
    n_private.param("crop_min_y", crop_min_y_, 0);
    n_private.param("crop_max_y", crop_max_y_, 480);
    n_private.param("display_wait_ms", display_wait_ms_, 3);
    n_private.param("min_workspace_x", min_workspace_x_, 0.0);
    n_private.param("min_workspace_y", min_workspace_y_, 0.0);
    n_private.param("min_workspace_z", min_workspace_z_, 0.0);
    n_private.param("max_workspace_x", max_workspace_x_, 0.0);
    n_private.param("max_workspace_y", max_workspace_y_, 0.0);
    n_private.param("max_workspace_z", max_workspace_z_, 0.0);
    std::string default_workspace_frame = "/torso_lift_link";
    n_private.param("workspace_frame", workspace_frame_,
                    default_workspace_frame);
    n_private.param("min_table_z", min_table_z_, -0.5);
    n_private.param("max_table_z", max_table_z_, 1.5);
    int num_hist_frames = 5;
    n_private.param("num_hist_frames", num_hist_frames, 5);
    motion_probs_.setNumHistFrames(num_hist_frames);
    double T_in = 3;
    double T_out = 5;
    n_private.param("T_in", T_in, 3.0);
    n_private.param("T_out", T_out, 5.0);
    motion_probs_.setT_inT_out(T_in, T_out);

    // Setup ros node connections
    sync_.registerCallback(&TabletopPushingPerceptionNode::sensorCallback,
                           this);
    push_pose_server_ = n_.advertiseService(
        "get_push_pose", &TabletopPushingPerceptionNode::getPushPose, this);
    table_location_server_ = n_.advertiseService(
        "get_table_location", &TabletopPushingPerceptionNode::getTableLocation,
        this);
  }

  void sensorCallback(const sensor_msgs::ImageConstPtr& img_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg,
                      const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // Convert images to OpenCV format
    cv::Mat color_frame(bridge_.imgMsgToCv(img_msg));
    cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));

    // Swap kinect color channel order
    cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);
    // cv::cvtColor(color_frame, color_frame, CV_RGB2Lab);

    // Transform point cloud into the correct frame and convert to PCL struct
    XYZPointCloud cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    pcl_ros::transformPointCloud(workspace_frame_, cloud, cloud, tf_);

    // Convert nans to zeros
    for (int r = 0; r < depth_frame.rows; ++r)
    {
      for (int c = 0; c < depth_frame.cols; ++c)
      {
        float cur_d = depth_frame.at<float>(r,c);
        if (isnan(cur_d))
          depth_frame.at<float>(r,c) = 0.0;
      }
    }

    // Select inner ROI of images to remove border issues
    int x_size = crop_max_x_ - crop_min_x_;
    int y_size = crop_max_y_ - crop_min_y_;
    cv::Rect roi(crop_min_x_, crop_min_y_, x_size, y_size);

    cv::Mat depth_region = depth_frame(roi);
    cv::Mat color_region = color_frame(roi);

    // Save internally for use in the service callback
    cur_color_frame_ = color_region.clone();
    cur_depth_frame_ = depth_region.clone();
    cur_point_cloud_ = cloud;
    have_depth_data_ = true;

    // Started via actionlib call
    if (tracking_)
      trackRegions(cur_color_frame_, cur_depth_frame_, cur_point_cloud_);
  }

  bool getTableLocation(LocateTable::Request& req, LocateTable::Response& res)
  {
    if ( have_depth_data_ )
    {
      res.table_centroid = getTablePlane(cur_color_frame_, cur_depth_frame_,
                                         cur_point_cloud_);
      if (res.table_centroid.pose.position.x == 0.0 &&
          res.table_centroid.pose.position.y == 0.0 &&
          res.table_centroid.pose.position.z == 0.0)
      {
        return false;
      }

    }
    else
    {
      ROS_ERROR_STREAM("Calling getTableLocation prior to receiving sensor data.");
      return false;
    }
    return true;
  }

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    if ( have_depth_data_ )
    {
      res = findPushPose(cur_color_frame_, cur_depth_frame_, cur_point_cloud_);
    }
    else
    {
      ROS_ERROR_STREAM("Calling getPushPose prior to receiving sensor data.");
      return false;
    }
    return true;
  }

  PushPose::Response findPushPose(cv::Mat visual_frame,
                                  cv::Mat depth_frame,
                                  XYZPointCloud& cloud)
  {

    // TODO: Choose a patch based on some simple criterian
    // TODO: Estimate the surface of the patch from the depth image
    // TODO: Extract the push pose as point in the center of that surface
    // TODO: Transform to be in the workspace_frame
    PushPose::Response res;
    PoseStamped p;
    res.push_pose = p;
    res.invalid_push_pose = false;
    return res;
  }

  //
  // Region tracking methods
  //

  void startTracker(const tabletop_pushing::SegTrackGoalConstPtr &goal)
  {
    tracker_initialized_ = false;
    tracking_ = true;

  }
  void stopTracker(const tabletop_pushing::SegTrackGoalConstPtr &goal)
  {
    tracker_initialized_ = false;
    tracking_ = false;
  }

  void trackRegions(cv::Mat& color_frame, cv::Mat& depth_frame,
                    XYZPointCloud& cloud)
  {
    if (!tracker_initialized_)
    {
      motion_probs_.initialize(color_frame, depth_frame);

      // Fit plane to table
      // TODO: Get extent as well
      table_centroid_ = getTablePlane(color_frame, depth_frame, cloud);
      tracker_initialized_ = true;
      return;
    }
    std::vector<cv::Mat> cur_probs = motion_probs_.update(color_frame, depth_frame);
    if (cur_probs.size() < 1) return;

    std::vector<cv::Mat> motion_prob_channels;
    cv::split(cur_probs[0], motion_prob_channels);
#ifdef DISPLAY_MOTION_PROBS
#ifdef DISPLAY_INPUT_IMAGES
    cv::imshow("bgr_frame", color_frame);
    cv::imshow("d_frame", depth_frame);
#endif // DISPLAY_INPUT_IMAGES
#ifdef DISPLAY_INTERMEDIATE_PROBS
    cv::imshow("b_motion_prob", motion_prob_channels[0]);
    cv::imshow("g_motion_prob", motion_prob_channels[1]);
    cv::imshow("r_motion_prob", motion_prob_channels[2]);
    cv::imshow("d_motion_prob", cur_probs[1]);
    cv::imshow("combined_motion_prob", motion_prob_channels[3]);
#endif // DISPLAY_INTERMEDIATE_PROBS
    cv::Mat motion_morphed(cur_probs[0].rows, cur_probs[0].cols, CV_32FC1);
    cv::Mat element(3, 3, CV_32FC1, cv::Scalar(1.0));
    cv::erode(motion_prob_channels[3], motion_morphed, element);
    cv::imshow("combined_motion_prob_clean", motion_morphed);
    cv::waitKey(display_wait_ms_);
#endif // DISPLAY_MOTION_PROBS
  }

  PoseStamped getTablePlane(cv::Mat& color_frame, cv::Mat& depth_frame,
                            XYZPointCloud& cloud)
  {
    // Filter Cloud to not look for table planes on the ground
    pcl::PointCloud<pcl::PointXYZ> cloud_z_filtered;
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setInputCloud(
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    z_pass.setFilterFieldName ("z");
    z_pass.setFilterLimits(min_table_z_, max_table_z_);
    z_pass.filter(cloud_z_filtered);

    // Segment the tabletop from the points using RANSAC plane fitting
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices plane_inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType (pcl::SACMODEL_PLANE);
    plane_seg.setMethodType (pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold (0.01);
    plane_seg.setInputCloud (
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_z_filtered));
    plane_seg.segment(plane_inliers, coefficients);

    // Check size of plane_inliers
    if (plane_inliers.indices.size() < 1)
    {
      ROS_WARN_STREAM("No points found by RANSAC plane fitting");
      PoseStamped p;
      p.pose.position.x = 0.0;
      p.pose.position.y = 0.0;
      p.pose.position.z = 0.0;
      p.header = cloud.header;
      return p;
    }

    // Extract the plane members into their own point cloud
    pcl::PointCloud<pcl::PointXYZ> plane_cloud;
    pcl::copyPointCloud(cloud_z_filtered, plane_inliers, plane_cloud);

    // Return plane centroid
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(plane_cloud, xyz_centroid);
    PoseStamped p;
    p.pose.position.x = xyz_centroid[0];
    p.pose.position.y = xyz_centroid[1];
    p.pose.position.z = xyz_centroid[2];
    p.header = cloud.header;
    ROS_INFO_STREAM("Table centroid is: ("
                    << p.pose.position.x << ", "
                    << p.pose.position.y << ", "
                    << p.pose.position.z << ")");
    // TODO: Get extent as well
    return p;
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
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Synchronizer<MySyncPolicy> sync_;
  actionlib::SimpleActionServer<tabletop_pushing::SegTrackAction> track_server_;
  sensor_msgs::CvBridge bridge_;
  tf::TransformListener tf_;
  ros::ServiceServer push_pose_server_;
  ros::ServiceServer table_location_server_;
  cv::Mat cur_color_frame_;
  cv::Mat cur_depth_frame_;
  XYZPointCloud cur_point_cloud_;
  ProbImageDifferencing motion_probs_;
  bool have_depth_data_;
  int crop_min_x_;
  int crop_max_x_;
  int crop_min_y_;
  int crop_max_y_;
  int display_wait_ms_;
  cv::RotatedRect cur_ellipse_;
  cv::RotatedRect delta_ellipse_;
  double min_workspace_x_;
  double max_workspace_x_;
  double min_workspace_y_;
  double max_workspace_y_;
  double min_workspace_z_;
  double max_workspace_z_;
  double min_table_z_;
  double max_table_z_;
  std::string workspace_frame_;
  PoseStamped table_centroid_;
  bool tracking_;
  bool tracker_initialized_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  TabletopPushingPerceptionNode perception_node(n);
  perception_node.spin();
}
