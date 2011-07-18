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
#include <utility>
#include <math.h>

#define DISPLAY_SUPERPIXELS 1
// #define DISPLAY_TRACKER_OUTPUT 1
// #define VOTE_FOR_REGION_ID 1
#define DISPLAY_MOVING_STUFF 1

using cpl_superpixels::getSuperpixelImage;
using tabletop_pushing::PushPose;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image> MySyncPolicy;

typedef std::vector<float> Descriptor;

const cv::RotatedRect EMPTY_ELLIPSE(cv::Point2f(0.0f, 0.0f),
                                    cv::Size2f(0.0f, 0.0f),
                                    0.0f);

bool operator==(cv::RotatedRect a, cv::RotatedRect b)
{
  return (a.center.x == b.center.x && a.center.y == b.center.y &&
          a.size.width == b.size.width && a.size.height == b.size.height);
}

bool operator!=(cv::RotatedRect a, cv::RotatedRect b)
{
  return !(a == b);
}

struct Flow
{
  Flow(int _x, int _y, int _dx, int _dy) : x(_x), y(_y), dx(_dx), dy(_dy)
  {
  }
  int x, y, dx, dy;
};

typedef std::pair<uchar, Flow> RegionMember;

class FeatureTracker
{
 public:
  FeatureTracker(std::string name, double hessian_thresh=250, int num_octaves=4,
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
        ROS_DEBUG_STREAM("Point is moving (" << sparse_flow[i].dx << ", "
                         << sparse_flow[i].dy << ")");
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

  void stop()
  {
    initialized_ = false;
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
      tracker_("i_tracker"),
      have_depth_data_(false), min_flow_thresh_(0), num_region_points_thresh_(1)
  {
    // Get parameters from the server
    ros::NodeHandle n_private("~");
    n_private.param("segment_k", k_, 500.0);
    n_private.param("segment_sigma", sigma_, 0.9);
    n_private.param("segment_min_size", min_size_, 30);
    n_private.param("segment_color_weight", wc_, 0.1);
    n_private.param("segment_depth_weight", wd_, 0.7);
    n_private.param("min_flow_thresh", min_flow_thresh_, 0);
    n_private.param("num_moving_points_per_region_thresh",
                    num_region_points_thresh_, 1);
    n_private.param("crop_min_x", crop_min_x_, 0);
    n_private.param("crop_max_x", crop_max_x_, 640);
    n_private.param("crop_min_y", crop_min_y_, 0);
    n_private.param("crop_max_y", crop_max_y_, 480);
    n_private.param("display_wait_ms", display_wait_ms_, 3);

    // Setup internal class stuff
    tracker_.setMinFlowThresh(min_flow_thresh_);

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
    cv::Mat color_frame(bridge_.imgMsgToCv(img_msg));
    cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));
    // Swap kinect color channel order
    cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);

    // TODO: Fill in gaps inside the depth data
    // Convert nans to zeros
    for (int r = 0; r < depth_frame.rows; ++r)
    {
      for (int c = 0; c < depth_frame.cols; ++c)
      {
        if (isnan(depth_frame.at<float>(r,c)))
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
    cur_color_frame_ = color_region;
    cur_depth_frame_ = depth_region;
    have_depth_data_ = true;

    // TODO: Add a service call to turn tracking on and off
    // TODO: Must deal with reseting and what not
    trackRegions(cur_color_frame_, cur_depth_frame_);
  }

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    if ( have_depth_data_ )
    {
      res = findPushPose(cur_color_frame_, cur_depth_frame_);
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
    cv::Mat regions = getSuperpixels(visual_frame, depth_frame);

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

  void trackRegions(cv::Mat& color_frame, cv::Mat& depth_frame)
  {
    cv::Mat regions = getSuperpixels(color_frame, depth_frame);

    if (!tracker_.isInitialized())
    {
      initRegionTracks(color_frame, depth_frame);
      return;
    }

    cv::Mat moving = updateRegionTracks(color_frame, depth_frame, regions);
    cv::RotatedRect el = computeEllipsoid2D(color_frame, moving);

    // Compute ellipsoid changes from previous frame and store as part of
    // current state
    if (cur_ellipse_ == EMPTY_ELLIPSE)
    {
      cur_ellipse_ = el;
    }
    else if (el != EMPTY_ELLIPSE)
    {
      // Calculate the deltas
      cv::RotatedRect delta(cv::Point2f(el.center.x - cur_ellipse_.center.x,
                                        el.center.y - cur_ellipse_.center.y),
                            cv::Size2f(el.size.width - cur_ellipse_.size.width,
                                       el.size.height - cur_ellipse_.size.height),
                            el.angle - cur_ellipse_.angle);
      delta_ellipse_ = delta;
    }
    else
    {
      // No detected movement so maintain keep the last one stationary
      delta_ellipse_ = EMPTY_ELLIPSE;
    }
  }

  void initRegionTracks(cv::Mat& color_frame, cv::Mat& depth_frame)
  {
    cv::Mat bw_frame(color_frame.rows, color_frame.cols, CV_8UC1);
    cv::cvtColor(color_frame, bw_frame, CV_BGR2GRAY);
    tracker_.initTracks(bw_frame);
  }

  cv::Mat updateRegionTracks(cv::Mat& color_frame, cv::Mat& depth_frame,
                             cv::Mat& regions)
  {
    cv::Mat bw_frame(color_frame.rows, color_frame.cols, CV_8UC1);
    cv::cvtColor(color_frame, bw_frame, CV_BGR2GRAY);
    std::vector<Flow> sparse_flow = tracker_.updateTracks(bw_frame);

    // Determine which regions are moving
    std::multimap<uchar, Flow> moving_regions;

    ROS_DEBUG_STREAM("Finding moving points");
    for (unsigned int i = 0; i < sparse_flow.size(); ++i)
    {
      // Filter out small flow
      if (sparse_flow[i].dx + sparse_flow[i].dy <= min_flow_thresh_)
        continue;
      // Add regions associated with moving points to the set
      // Store the flow associated with each region
      moving_regions.insert(RegionMember
                            (regions.at<uchar>(sparse_flow[i].y,
                                               sparse_flow[i].x),
                             sparse_flow[i]));
    }

    // TODO: Remove table plane regions from possible moving regions
    // Create a mask of the moving regions drawn
    cv::Mat moving_regions_mask(regions.rows, regions.cols, CV_8UC1,
                                cv::Scalar(0));
    for (int r = 0; r < regions.rows; ++r)
    {
      for (int c = 0; c < regions.cols; ++c)
      {
        // Test if the region value at r,c is moving
        if (moving_regions.count(regions.at<uchar>(r,c)) >=
            num_region_points_thresh_)
        {
          moving_regions_mask.at<uchar>(r,c) = 255;
        }
      }
    }

#ifdef DISPLAY_MOVING_STUFF
    // Create a color image of the moving parts using the mask
    cv::Mat moving_regions_img;
    color_frame.copyTo(moving_regions_img, moving_regions_mask);
    cv::imshow("Mask", moving_regions_mask);
    cv::imshow("Moving regions", moving_regions_img);
    cv::waitKey(display_wait_ms_);
#endif // DISPLAY_MOVING_STUFF

    return moving_regions_mask;
  }

  //
  // Controller State representations
  //

  cv::RotatedRect computeEllipsoid2D(cv::Mat& color_frame, cv::Mat moving)
  {
    cv::Mat contour_img;
    color_frame.copyTo(contour_img, moving);
    std::vector<std::vector<cv::Point> > moving_contours;
    moving_contours.clear();
    // NOTE: This method makes changes to the "moving" image
    cv::findContours(moving, moving_contours, cv::RETR_EXTERNAL,
                     CV_CHAIN_APPROX_NONE);

    // TODO: Compute a single ellipse to describe the motion?
    std::vector<cv::RotatedRect> els;
    // Compute secondary features from these
    for (unsigned int i = 0; i < moving_contours.size(); ++i)
    {
      // Get moments
      cv::Mat pt_mat(moving_contours[i]);
      cv::Moments m;
      m = cv::moments(pt_mat);
      // Fit an ellipse
      cv::RotatedRect el = cv::fitEllipse(pt_mat);
      ROS_INFO_STREAM("ellipse " << i << " has center (" << el.center.x
                      << ", " << el.center.y << ")"
                      <<  " and size (" << el.size.width << ", "
                      << el.size.height << ")");
      // Draw ellipse for display purposes
      cv::Scalar ellipse_color(255, 0, 0);
      cv::ellipse(contour_img, el, ellipse_color, 2);
      els.push_back(el);
    }
    cv::Scalar object_contour_color(0, 0, 255);
    if (moving_contours.size() > 0)
    {
      cv::drawContours(contour_img, moving_contours, -1,
                       object_contour_color, 2);
    }

    cv::imshow("contour window", contour_img);
    cv::waitKey();
    if (moving_contours.size() > 0)
      return els[0];
    return EMPTY_ELLIPSE;
  }

  void computeEllipsoid2D(cv::Mat& color_frame, cv::Mat& depth_frame,
                          cv::Mat moving)
  {
    // TODO: Determine return type to use (3D Pose and axes sizes)
  }


  //
  // Visual Feature Extraction
  //

  cv::Mat getSuperpixels(cv::Mat& color_frame, cv::Mat& depth_frame)
  {
    // TODO: Use normal estimation for segmentation
    cv::Mat display_regions;
    int num_regions = 0;
    cv::Mat regions = getSuperpixelImage(color_frame, depth_frame,
                                         num_regions, display_regions,
                                         sigma_, k_, min_size_, wc_, wd_);
    ROS_INFO_STREAM("Computed " << num_regions << " regions");

#ifdef DISPLAY_SUPERPIXELS
    cv::Mat depth_display = depth_frame.clone();
    double max_val = 1.0;
    cv::minMaxLoc(depth_display, NULL, &max_val);
    if (max_val > 0.0)
    {
      depth_display /= max_val;
    }
    cv::imshow("color_frame", color_frame);
    cv::imshow("depth_frame", depth_display);
    // cv::imshow("depth_frame_real", depth_frame);
    // cv::imshow("depth_frame_region", depth_region);
    cv::imshow("regions", display_regions);
    // cv::imshow("real_regions", regions);
#endif // DISPLAY_SUPERPIXELS
    return regions;
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
  cv::Mat cur_color_frame_;
  cv::Mat cur_depth_frame_;
  FeatureTracker tracker_;
  bool have_depth_data_;
  double k_;
  double sigma_;
  int min_size_;
  double wc_;
  double wd_;
  int min_flow_thresh_;
  int num_region_points_thresh_;
  int crop_min_x_;
  int crop_max_x_;
  int crop_min_y_;
  int crop_max_y_;
  int display_wait_ms_;
  cv::RotatedRect cur_ellipse_;
  cv::RotatedRect delta_ellipse_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  TabletopPushingPerceptionNode perception_node(n);
  perception_node.spin();
}
