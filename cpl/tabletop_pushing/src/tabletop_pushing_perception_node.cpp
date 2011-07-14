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
#include <pcl/features/normal_3d.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

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
#include <set>
#include <map>
#include <queue>
#include <string>
#include <sstream>
#include <fstream>
#include <utility>
#include <math.h>

#define CALL_PUSH_POSE_ON_CALLBCK
#define DISPLAY_SUPERPIXELS
// #define DISPLAY_TRACKER_OUTPUT
#define USE_DEPTH_ROI

#define R_INDEX 2
#define G_INDEX 1
#define B_INDEX 0

using cpl_superpixels::getSuperpixelImage;
using tabletop_pushing::PushPose;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image> MySyncPolicy;

typedef std::vector<float> Descriptor;
typedef unsigned short uint8;
struct Flow
{
  Flow(int _x, int _y, int _dx, int _dy) : x(_x), y(_y), dx(_dx), dy(_dy)
  {
  }
  int x, y, dx, dy;
};

typedef std::pair<int, Flow> RegionMember;

class FeatureTracker
{
 public:
  FeatureTracker(std::string name, double hessian_thresh=500, int num_octaves=4,
                 int num_layers=2, bool extended=true) :
      surf_(hessian_thresh, num_octaves, num_layers, extended),
      initialized_(false), ratio_threshold_(0.3), window_name_(name),
      min_flow_thresh_(0)
  {
    prev_keypoints_.clear();
    cur_keypoints_.clear();
    prev_descriptors_.clear();
    cur_descriptors_.clear();
  }

  //
  // Main tracking logic functions
  //

  void initTracks(cv::Mat& frame)
  {
    updateCurrentDescriptors(frame);
    prev_keypoints_ = cur_keypoints_;
    prev_descriptors_ = cur_descriptors_;
    initialized_ = true;
  }

  std::vector<Flow> updateTracks(cv::Mat& frame)
  {
    cur_keypoints_.clear();
    cur_descriptors_.clear();
    updateCurrentDescriptors(frame);

    std::vector<int> matches_cur;
    std::vector<int> matches_prev;
    std::vector<Flow> sparse_flow;
    matches_cur.clear();
    matches_prev.clear();

    // Find nearest neighbors with previous descriptors
    findMatches(cur_descriptors_, prev_descriptors_, matches_cur, matches_prev);
    ROS_DEBUG_STREAM(window_name_ << ": num feature matches: "
                    << matches_cur.size());
    int moving_points = 0;
    for (unsigned int i = 0; i < matches_cur.size(); i++)
    {
      int dx = prev_keypoints_[matches_prev[i]].pt.x -
          cur_keypoints_[matches_cur[i]].pt.x;
      int dy = prev_keypoints_[matches_prev[i]].pt.y -
          cur_keypoints_[matches_cur[i]].pt.y;
      sparse_flow.push_back(Flow(cur_keypoints_[matches_cur[i]].pt.x,
                                 cur_keypoints_[matches_cur[i]].pt.y,
                                 dx, dy));
      if (sparse_flow[i].dx + sparse_flow[i].dy > min_flow_thresh_)
        moving_points++;
    }
    ROS_DEBUG_STREAM(window_name_ << ": num moving points: " << moving_points);

#ifdef DISPLAY_TRACKER_OUTPUT
    cv::Mat display_frame(frame.rows, frame.cols, CV_8UC3);;
    cv::cvtColor(frame, display_frame, CV_GRAY2BGR);
    for (unsigned int i = 0; i < matches_cur.size(); i++)
    {
      if (sparse_flow[i].dx + sparse_flow[i].dy > min_flow_thresh_)
      {
        cv::line(display_frame,
                 prev_keypoints_[matches_prev[i]].pt,
                 cur_keypoints_[matches_cur[i]].pt,
                 cv::Scalar(0,0,255), 1);
      }
    }

    cv::imshow(window_name_, display_frame);
    char c = cv::waitKey(3);
#endif // DISPLAY_TRACKER_OUTPUT

    prev_keypoints_ = cur_keypoints_;
    prev_descriptors_ = cur_descriptors_;
    return sparse_flow;
  }

  //
  // Feature Matching Functions
  //

  /*
   * SSD
   *
   * @short Computes the squareroot of squared differences
   * @param a First descriptor
   * @param b second descriptor
   * @return value of squareroot of squared differences
   */
  double SSD(Descriptor& a, Descriptor& b)
  {
    double diff = 0;

    for (unsigned int i = 0; i < a.size(); ++i) {
      float delta = a[i] - b[i];
      diff += delta*delta;
    }

    return diff;
  }

  /*
   * ratioTest
   *
   * @short Computes the  ratio test described in Lowe 2004
   * @param a Descriptor from the first image to compare
   * @param bList List of descriptors from the second image
   * @param threshold Threshold value for ratioTest comparison
   *
   * @return index of the best match, -1 if no match ratio is less than threshold
   */
  int ratioTest(Descriptor& a, std::vector<Descriptor>& bList, double threshold)
  {
    double bestScore = 1000000;
    double secondBest = 1000000;
    int bestIndex = -1;

    for (unsigned int b = 0; b < bList.size(); ++b) {
      double score = 0;
      score = SSD(a, bList[b]);

      if (score < bestScore) {
        secondBest = bestScore;
        bestScore = score;
        bestIndex = b;
      } else if (score < secondBest) {
        secondBest = score;
      }
      if ( bestScore / secondBest > threshold) {
        bestIndex = -1;
      }

    }

    return bestIndex;
  }

  /**
   * findMatches
   *
   * @param descriptors1 List of descriptors from image 1
   * @param descriptors2 List of descriptors from image 2
   * @param matches1 Indexes of matching points in image 1 (Returned)
   * @param matches2 Indexes of matching points in image 2 (Returned)
   */
  void findMatches(std::vector<Descriptor>& descriptors1,
                   std::vector<Descriptor>& descriptors2,
                   std::vector<int>& matches1, std::vector<int>& matches2)
  {
    // Determine matches using the Ratio Test method from Lowe 2004
    for (unsigned int a = 0; a < descriptors1.size(); ++a) {
      const int bestIndex = ratioTest(descriptors1[a], descriptors2,
                                      ratio_threshold_);
      if (bestIndex != -1) {
        matches1.push_back(a);
        matches2.push_back(bestIndex);
      }
    }

    // Check that the matches are unique going the other direction
    for (unsigned int x = 0; x < matches2.size();) {
      const int bestIndex = ratioTest(descriptors2[matches2[x]],
                                      descriptors1, ratio_threshold_);
      if (bestIndex != matches1[x]) {
        matches1.erase(matches1.begin()+x);
        matches2.erase(matches2.begin()+x);
      } else {
        x++;
      }
    }

  }

 protected:

  //
  // Helper Functions
  //

  void updateCurrentDescriptors(cv::Mat& frame)
  {
    std::vector<float> raw_descriptors;
    try
    {
      surf_(frame, cv::Mat(), cur_keypoints_, raw_descriptors);
      for (unsigned int i = 0; i < raw_descriptors.size(); i += 128)
      {
        Descriptor d(raw_descriptors.begin() + i,
                     raw_descriptors.begin() + i + 128);
        cur_descriptors_.push_back(d);
      }
    }
    catch(cv::Exception e)
    {
      std::cerr << e.err << std::endl;
    }
  }

 public:

  //
  // Getters & Setters
  //

  bool isInitialized() const
  {
    return initialized_;
  }

  void setMinFlowThresh(int min_thresh)
  {
    min_flow_thresh_= min_thresh;
  }

 protected:
  std::vector<cv::KeyPoint> prev_keypoints_;
  std::vector<cv::KeyPoint> cur_keypoints_;
  std::vector<Descriptor> prev_descriptors_;
  std::vector<Descriptor> cur_descriptors_;
  cv::SURF surf_;
  bool initialized_;
  double ratio_threshold_;
  std::string window_name_;
  int min_flow_thresh_;
};

class TabletopPushingPerceptionNode
{
 public:
  TabletopPushingPerceptionNode(ros::NodeHandle &n) :
      n_(n),
      image_sub_(n, "color_image_topic", 1),
      depth_sub_(n, "depth_image_topic", 1),
      sync_(MySyncPolicy(1), image_sub_, depth_sub_),
      tf_(),
      r_tracker_("r_tracker"), g_tracker_("g_tracker"), b_tracker_("b_tracker"),
      have_depth_data_(false), min_flow_thresh_(0)
  {
    // Get parameters from the server
    ros::NodeHandle n_private("~");
    n_private.param("segment_k", k_, 500.0);
    n_private.param("segment_sigma", sigma_, 0.9);
    n_private.param("segment_min_size", min_size_, 30);
    n_private.param("segment_color_weight", wc_, 0.1);
    n_private.param("segment_depth_weight", wd_, 0.7);
    n_private.param("min_flow_thresh", min_flow_thresh_, 0);

    // Setup internal class stuff
    r_tracker_.setMinFlowThresh(min_flow_thresh_);
    g_tracker_.setMinFlowThresh(min_flow_thresh_);
    b_tracker_.setMinFlowThresh(min_flow_thresh_);

    // Setup ros node connections
    sync_.registerCallback(&TabletopPushingPerceptionNode::sensorCallback,
                           this);
    push_pose_server_ = n_.advertiseService(
        "get_push_pose", &TabletopPushingPerceptionNode::getPushPose, this);
  }

  void sensorCallback(const sensor_msgs::ImageConstPtr& img_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg)
  {
    // Convert images to OpenCV format
    cv::Mat visual_frame(bridge_.imgMsgToCv(img_msg));
    cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));

    // Save internally for use in the service callback
    cur_visual_frame_ = visual_frame;
    cur_depth_frame_ = depth_frame;
    have_depth_data_ = true;
#ifdef CALL_PUSH_POSE_ON_CALLBCK
    PushPose::Response p = findPushPose(cur_visual_frame_, cur_depth_frame_);
#endif // CALL_PUSH_POSE_ON_CALLBCK
  }

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    if ( have_depth_data_ )
    {
      res = findPushPose(cur_visual_frame_, cur_depth_frame_);
    }
    else
    {
      ROS_ERROR_STREAM("Calling getPushPose prior to receiving sensor data.");
      return false;
    }
    return true;
  }

  PushPose::Response findPushPose(cv::Mat& visual_frame,
                                  cv::Mat& depth_frame)
  {
    return superpixelFindPushPose(visual_frame, depth_frame);
  }

  // TODO: Pull out the superpixel stuff to be separate for use in findPose and
  // in online prediction
  PushPose::Response superpixelFindPushPose(cv::Mat& color_frame,
                                            cv::Mat& depth_frame)
  {
    // Swap kinect color channel order
    cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);
    // TODO: Use normal estimation for segmentation
    // TODO: Fill in gaps inside the depth data.
    int num_regions = 0;

    // Select inner ROI of images to remove border issues?
#ifdef USE_DEPTH_ROI
    // TODO: Set these values from the parameter server
    int y_start = 31;
    int x_start = 0;
    int x_end = 580 - x_start;
    int y_end = depth_frame.rows - y_start;
    cv::Rect roi(x_start, y_start, x_end, y_end);
    cv::Mat depth_region = depth_frame(roi);
    cv::Mat color_region = color_frame(roi);
    cv::Mat display_regions;
    // cv::Mat regions = getSuperpixelImage(color_region, depth_region,
    //                                      num_regions, display_regions,
    //                                      sigma_, k_, min_size_, wc_, wd_);
    cv::Mat regions = getSuperpixelImage(color_region,
                                         num_regions, display_regions,
                                         sigma_, k_, min_size_);
#else // USE_DEPTH_ROI
    cv::Mat regions = getSuperpixelImage(color_frame, depth_frame, num_regions,
                                         display_regions, sigma_, k_, min_size_,
                                         wc_, wd_);

#endif // USE_DEPTH_ROI
    ROS_INFO_STREAM("Computed " << num_regions << " regions");

#ifdef DISPLAY_SUPERPIXELS
#ifdef USE_DEPTH_ROI
    cv::Mat depth_display = depth_region.clone();
    double max_val = 1.0;
    cv::minMaxLoc(depth_display, NULL, &max_val);
    if (max_val > 0.0)
    {
      depth_display /= max_val;
    }
    cv::imshow("color_frame", color_region);
    cv::imshow("depth_region", depth_display);
#else // USE_DEPTH_ROI
    cv::Mat depth_display = depth_frame.clone();
    double max_val = 1.0;
    cv::minMaxLoc(depth_display, NULL, &max_val);
    if (max_val > 0.0)
    {
      depth_display /= max_val;
    }
    cv::imshow("color_frame", color_frame);
    cv::imshow("depth_scaled_frame", depth_display);
#endif // USE_DEPTH_ROI
    cv::imshow("regions", display_regions);
    // cv::imshow("real_regions", regions);
#endif // DISPLAY_SUPERPIXELS

#ifdef USE_DEPTH_ROI
    if (!r_tracker_.isInitialized())
    {
      initRegionTracks(color_region, depth_region);
    }
    else
    {
      updateRegionTracks(color_region, depth_region, regions);
    }
#else // USE_DEPTH_ROI
    if (!r_tracker_.isInitialized())
    {
      initRegionTracks(color_frame, depth_frame);
    }
    else
    {
      updateRegionTracks(color_frame, depth_frame, regions);
    }
#endif // USE_DEPTH_ROI

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

  //
  // Region tracking methods
  //
  void initRegionTracks(cv::Mat& color_frame, cv::Mat& depth_frame)
  {
    // cv::Mat bw_frame(color_frame.rows, color_frame.cols, CV_8UC1);
    // cv::cvtColor(color_frame, bw_frame, CV_BGR2GRAY);
    // tracker_.initTracks(bw_frame);
    vector<cv::Mat> channels;
    cv::split(color_frame, channels);
    r_tracker_.initTracks(channels[R_INDEX]);
    g_tracker_.initTracks(channels[G_INDEX]);
    b_tracker_.initTracks(channels[B_INDEX]);
  }

  void updateRegionTracks(cv::Mat& color_frame, cv::Mat& depth_frame,
                          cv::Mat& regions)
  {
    // cv::Mat bw_frame(color_frame.rows, color_frame.cols, CV_8UC1);
    // cv::cvtColor(color_frame, bw_frame, CV_BGR2GRAY);
    vector<cv::Mat> channels;
    cv::split(color_frame, channels);
    std::vector<Flow> r_sparse_flow = r_tracker_.updateTracks(channels[R_INDEX]);
    std::vector<Flow> g_sparse_flow = g_tracker_.updateTracks(channels[G_INDEX]);
    std::vector<Flow> b_sparse_flow = b_tracker_.updateTracks(channels[B_INDEX]);

    // Determine which regions are moving
    std::multimap<uchar, Flow> moving_regions;

    ROS_INFO_STREAM("Finding moving points");
    int points_added = 0;
    for (unsigned int i = 0; i < r_sparse_flow.size(); ++i)
    {
      // Filter out small flow
      if (r_sparse_flow[i].dx + r_sparse_flow[i].dy <= min_flow_thresh_)
        continue;
      // Add regions associated with moving points to the set
      // Store the flow associated with each region
      moving_regions.insert(RegionMember
                            (regions.at<uchar>(r_sparse_flow[i].y,
                                               r_sparse_flow[i].x),
                             r_sparse_flow[i]));
      points_added++;
    }
    for (unsigned int i = 0; i < g_sparse_flow.size(); ++i)
    {
      // Filter out small flow
      if (g_sparse_flow[i].dx + g_sparse_flow[i].dy <= min_flow_thresh_)
        continue;

      moving_regions.insert(RegionMember
                            (regions.at<uchar>(g_sparse_flow[i].y,
                                               g_sparse_flow[i].x),
                             g_sparse_flow[i]));
      points_added++;
    }

    for (unsigned int i = 0; i < b_sparse_flow.size(); ++i)
    {
      // Filter out small flow
      if (b_sparse_flow[i].dx + b_sparse_flow[i].dy <= min_flow_thresh_)
        continue;

      moving_regions.insert(RegionMember
                            (regions.at<uchar>(b_sparse_flow[i].y,
                                               b_sparse_flow[i].x),
                             b_sparse_flow[i]));
      points_added++;
    }

    // Create an image with only moving regions drawn
    // TODO: Change this to be a color image of the moving one?
    cv::Mat moving_regions_img(regions.rows, regions.cols, CV_8UC1,
                               cv::Scalar(0));
    int painted = 0;
    for (int r = 0; r < regions.rows; ++r)
    {
      for (int c = 0; c < regions.cols; ++c)
      {
        // Test if the region value at r,c is moving
        if (moving_regions.find(regions.at<uchar>(r,c)) !=
            moving_regions.end())
        {
          try
          {
            moving_regions_img.at<uchar>(r,c) = 255;
            painted++;
          }
          catch(cv::Exception e)
          {
            ROS_ERROR_STREAM("Error trying to set image value: " << e.err);
          }
        }
      }
    }
    cv::imshow("Moving regions", moving_regions_img);
    cv::waitKey(3);

    // TODO: Determine if there is a single moving region or multiple regions
    // Compute secondary features from these
  }

  //
  // Controller State representations
  //
  void computeEllipsoid2D(cv::Mat& regions, std::vector<int>& active)
  {
  }

  void computeEllipsoid3D(cv::Mat& regions, cv::Mat& depth_frame,
                          std::vector<int>& active)
  {
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
  message_filters::Synchronizer<MySyncPolicy> sync_;
  sensor_msgs::CvBridge bridge_;
  tf::TransformListener tf_;
  ros::ServiceServer push_pose_server_;
  cv::Mat cur_visual_frame_;
  cv::Mat cur_depth_frame_;
  // FeatureTracker tracker_;
  FeatureTracker r_tracker_;
  FeatureTracker g_tracker_;
  FeatureTracker b_tracker_;
  bool have_depth_data_;
  double k_;
  double sigma_;
  int min_size_;
  double wc_;
  double wd_;
  int min_flow_thresh_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  TabletopPushingPerceptionNode perception_node(n);
  perception_node.spin();
}
