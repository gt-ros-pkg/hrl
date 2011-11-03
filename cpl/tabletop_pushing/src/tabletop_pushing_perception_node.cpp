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
#include <actionlib/server/simple_action_server.h>

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
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/feature.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// tabletop_pushing
#include <tabletop_pushing/PushPose.h>
#include <tabletop_pushing/LocateTable.h>
#include <tabletop_pushing/SegTrackAction.h>

#include <tabletop_pushing/extern/graphcut/graph.h>
#include <tabletop_pushing/extern/graphcut/energy.h>
#include <tabletop_pushing/extern/graphcut/GCoptimization.h>

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

// Debugging IFDEFS
// #define DISPLAY_INPUT_COLOR 1
// #define DISPLAY_INPUT_DEPTH 1
// #define DISPLAY_WORKSPACE_MASK 1
// #define DISPLAY_OPTICAL_FLOW 1
// #define DISPLAY_PLANE_ESTIMATE 1
// #define DISPLAY_UV 1
// #define DISPLAY_GRAPHCUT 1
// #define VISUALIZE_GRAPH_WEIGHTS 1
// #define VISUALIZE_GRAPH_EDGE_WEIGHTS 1
// #define VISUALIZE_ARM_GRAPH_WEIGHTS 1
// #define VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS 1
// #define DISPLAY_ARM_CIRCLES 1
// #define DISPLAY_TABLE_DISTANCES 1
// #define DISPLAY_FLOW_FIELD_CLUSTERING 1
// #define DISPLAY_TRACKER_OUTPUT 1
// #define WRITE_INPUT_TO_DISK 1
// #define WRITE_CUTS_TO_DISK 1
// #define WRITE_FLOWS_TO_DISK 1
// #define WRITE_ARM_CUT_TO_DISK 1

// Functional IFDEFS
#define MEDIAN_FILTER_FLOW 1
#define USE_WORKSPACE_MASK_FOR_ARM 1
// #define AUTO_FLOW_CLUSTER 1
// #define USE_TABLE_COLOR_ESTIMATE 1
#define SCALE_AFFINE_DISTANCES 1
#define CLUSTER_AFFINE_WITH_IMAGE_LOCS 1

using tabletop_pushing::PushPose;
using tabletop_pushing::LocateTable;
using geometry_msgs::PoseStamped;
using geometry_msgs::PointStamped;
using geometry_msgs::Pose2D;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2> MySyncPolicy;
typedef Graph<float, float, float> GraphType;
typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

inline float max(const float a, const double b)
{
  return std::max(static_cast<double>(a), b);
}

inline float min(const float a, const double b)
{
  return std::min(static_cast<double>(a), b);
}

void displayOpticalFlow(cv::Mat& color_frame, cv::Mat& flow_u, cv::Mat& flow_v,
                        float mag_thresh)
{
  cv::Mat flow_thresh_disp_img(color_frame.size(), CV_8UC3);
  flow_thresh_disp_img = color_frame.clone();
  for (int r = 0; r < flow_thresh_disp_img.rows; ++r)
  {
    for (int c = 0; c < flow_thresh_disp_img.cols; ++c)
    {
      float u = flow_u.at<float>(r,c);
      float v = flow_u.at<float>(r,c);
      if (std::sqrt(u*u+v*v) > mag_thresh)
      {
        cv::line(flow_thresh_disp_img, cv::Point(c,r), cv::Point(c-u, r-v),
                 cv::Scalar(0,255,0));
      }
    }
  }
  std::vector<cv::Mat> flows;
  cv::imshow("flow_disp", flow_thresh_disp_img);
#ifdef DISPLAY_UV
  cv::imshow("u", flow_u);
  cv::imshow("v", flow_v);
#endif // DISPLAY_UV
}

class ArmModel
{
 public:
  ArmModel() : l_hand_on(false),  r_hand_on(false), l_arm_on(false),
               r_arm_on(false)
  {
    hands.clear();
    arms.clear();
  }
  unsigned int size()
  {
    return 2;
  }

  std::vector<cv::Point> operator[](unsigned int i)
  {
    if (i == 0)
      return hands;
    if (i > 1)
    {
      std::stringstream err_msg;
      err_msg << "Index argument: " << i << " is out of range in class ArmModel";
      throw std::out_of_range(err_msg.str());
    }
    return arms;
  }

  /**
   * Return the distance to the closest point on the arm from an image point p
   *
   * @param p The point in the image
   *
   * @return The distance to the closest point on any in-image arm
   */
  float distanceToArm(cv::Point2f p, cv::Mat& depth_frame)
  {
    float l_dist = distanceToArm(p, l_chain, depth_frame);
    float r_dist = distanceToArm(p, r_chain, depth_frame);
    if (l_dist < 0 && r_dist < 0) return -1.0f;
    if (l_dist < 0)
    {
      return r_dist;
    }
    if (r_dist < 0)
    {
      return l_dist;
    }
    return min(l_dist, r_dist);
  }

  float distanceToArm(cv::Point2f p, std::vector<cv::Point>& chain,
                      cv::Mat& depth_frame)
  {
    if (chain.size() == 0)
    {
      return -1.0f;
    }
    float min_dist = 640.0f*480.0f;
    for (unsigned int i = 1; i < chain.size(); ++i)
    {
      float d_i = pointLineDistance(p, chain[i-1] , chain[i], depth_frame);
      if (d_i < min_dist)
        min_dist = d_i;
    }
    return min_dist;
  }

  float pointLineDistance(cv::Point2f p, cv::Point2f l0, cv::Point2f l1,
                          cv::Mat& depth_frame)
  {
    if (l0.x == l1.x)
    {
      cv::Point2f q(l0.x, p.y);
      float l_max_x = max(l0.x, l1.x);
      float l_min_x = min(l0.x, l1.x);
      if (p.y > l_max_x)
      {
        q.y = l_max_x;
      }
      else if (p.y < l_min_x)
      {
        q.y = l_min_x;
      }
      return pointPointDistance(p,q,depth_frame);
    }
    cv::Point2f x0;
    cv::Point2f x1;

    if (l0.x < l1.x)
    {
      x0 = l0;
      x1 = l1;
    }
    else
    {
      x0 = l1;
      x1 = l0;
    }
    cv::Point2f v = x1 - x0;
    cv::Point2f w = p - x0;

    float c0 = w.x*v.x+w.y*v.y;
    float c1 = v.x*v.x+v.y*v.y;
    float b = c0/c1;

    cv::Point2f q = x0 + b*v;

    float d = pointPointDistance(p,q,depth_frame);
    if (c0 <= 0 || q.x < x0.x)
    {
      d = pointPointDistance(p,x0,depth_frame);
    }
    if (c1 <= 0 || q.x > x1.x)
    {
      d = pointPointDistance(p,x1,depth_frame);
    }
    return d;
  }

  float pointPointDistance(cv::Point2f& p, cv::Point2f& q, cv::Mat& depth_frame)
  {
    // return hypot(p.x-q.x,p.y-q.y);
    // TODO: Use 3D distance between the two points
    const float d_x = p.x-q.x;
    const float d_y = p.y-q.y;
    // const float d_d = (depth_frame.at<float>(p.y,p.x)-
    //                    depth_frame.at<float>(q.y,q.x));
    // return sqrt(d_x*d_x + d_y*d_y + d_d*d_d);
    return sqrt(d_x*d_x + d_y*d_y);
  }

  std::vector<cv::Point> hands;
  std::vector<cv::Point> arms;
  std::vector<cv::Point> l_chain;
  std::vector<cv::Point> r_chain;
  bool l_hand_on;
  bool r_hand_on;
  bool l_arm_on;
  bool r_arm_on;
};

class AffineFlowMeasure
{
 public:
  AffineFlowMeasure(int x_ = 0, int y_ = 0, int u_ = 0, int v_= 0,
                    int label_=-1) : x(x_), y(y_), u(u_), v(v_), label(label_)
  {
  }

  cv::Mat X()
  {
    cv::Mat X(3, 1, CV_32FC1);
    X.at<float>(0,0) = x;
    X.at<float>(1,0) = y;
    X.at<float>(2,0) = 1.0;
    return X;
  }

  bool operator==(AffineFlowMeasure b)
  {
    return (b.x == x && b.y == y);
  }

  float operator-(AffineFlowMeasure b) const
  {
    const float d_x = x-b.x;
    const float d_y = y-b.y;
    return std::sqrt(d_x*d_x+d_y*d_y);
  }
  // Members
  float x;
  float y;
  float u;
  float v;
  int label;
  cv::Mat a;
};

typedef std::deque<AffineFlowMeasure> AffineFlowMeasures;
typedef std::vector<float> Descriptor;

class LKFlowReliable
{
 public:
  LKFlowReliable(int win_size = 5, int num_levels = 4) :
      win_size_(win_size), max_level_(num_levels-1)
  {
    // Create derivative kernels for flow calculation
    cv::getDerivKernels(dy_kernel_, dx_kernel_, 1, 0, CV_SCHARR, true, CV_32F);
    cv::flip(dy_kernel_, dy_kernel_, -1);
    g_kernel_ = cv::getGaussianKernel(3, 2.0, CV_32F);
    optic_g_kernel_ = cv::getGaussianKernel(5, 10.0, CV_32F);
    cv::transpose(dy_kernel_, dx_kernel_);
  }

  virtual ~LKFlowReliable()
  {
  }

  std::vector<cv::Mat> operator()(cv::Mat& cur_color_frame,
                                  cv::Mat& prev_color_frame)
  {
    // Convert to grayscale
    cv::Mat tmp_bw(cur_color_frame.size(), CV_8UC1);
    cv::Mat cur_bw(cur_color_frame.size(), CV_32FC1);
    cv::Mat prev_bw(prev_color_frame.size(), CV_32FC1);
    cv::cvtColor(cur_color_frame, tmp_bw, CV_BGR2GRAY);
    tmp_bw.convertTo(cur_bw, CV_32FC1, 1.0/255, 0);
    cv::cvtColor(prev_color_frame, tmp_bw, CV_BGR2GRAY);
    tmp_bw.convertTo(prev_bw, CV_32FC1, 1.0/255, 0);
    return hierarchy(cur_bw, prev_bw);
  }

  std::vector<cv::Mat> hierarchy(cv::Mat& f2, cv::Mat& f1)
  {
    const int divisor = std::pow(2,max_level_);
    cv::Mat Dx(f2.rows/divisor, f2.cols/divisor, CV_32FC1, cv::Scalar(0.0));
    cv::Mat Dy = Dx.clone();
    std::vector<cv::Mat> g1s;
    std::vector<cv::Mat> g2s;
    cv::buildPyramid(f1, g1s, max_level_);
    cv::buildPyramid(f2, g2s, max_level_);
    std::vector<cv::Mat> flow_outs;

    for (int l = max_level_; l >= 0; --l)
    {
      if (l != max_level_)
      {
        Dx = expand(Dx);
        Dy = expand(Dy);
      }
      cv::Mat W = warp(g1s[l], Dx, Dy);
      flow_outs = baseLK(g2s[l], W);
      Dx = Dx + flow_outs[0];
      Dy = Dy + flow_outs[1];
      Dx = smooth(Dx);
      Dy = smooth(Dy);
    }
    std::vector<cv::Mat> total_outs;
    total_outs.push_back(Dx);
    total_outs.push_back(Dy);
    return total_outs;
  }

  std::vector<cv::Mat> baseLK(cv::Mat& cur_bw, cv::Mat& prev_bw)
  {
    cv::Mat Ix(cur_bw.size(), CV_32FC1);
    cv::Mat Iy(cur_bw.size(), CV_32FC1);
    cv::Mat cur_blur(cur_bw.size(), cur_bw.type());
    cv::Mat prev_blur(prev_bw.size(), prev_bw.type());
    cv::filter2D(cur_bw, cur_blur, CV_32F, g_kernel_);
    cv::filter2D(prev_bw, prev_blur, CV_32F, g_kernel_);
    cv::Mat It = cur_blur - prev_blur;

    // Get image derivatives
    cv::filter2D(cur_bw, Ix, CV_32F, dx_kernel_);
    cv::filter2D(cur_bw, Iy, CV_32F, dy_kernel_);

    int win_radius = win_size_/2;
    cv::Mat flow_u(cur_bw.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat flow_v(cur_bw.size(), CV_32FC1, cv::Scalar(0.0));
    for (int r = win_radius; r < Ix.rows-win_radius; ++r)
    {
      for (int c = win_radius; c < Ix.cols-win_radius; ++c)
      {
        float sIxx = 0.0;
        float sIyy = 0.0;
        float sIxy = 0.0;
        float sIxt = 0.0;
        float sIyt = 0.0;
        for (int y = r-win_radius; y <= r+win_radius; ++y)
        {
          for (int x = c-win_radius; x <= c+win_radius; ++x)
          {
            sIxx += Ix.at<float>(y,x)*Ix.at<float>(y,x);
            sIyy += Iy.at<float>(y,x)*Iy.at<float>(y,x);
            sIxy += Ix.at<float>(y,x)*Iy.at<float>(y,x);
            sIxt += Ix.at<float>(y,x)*It.at<float>(y,x);
            sIyt += Iy.at<float>(y,x)*It.at<float>(y,x);
          }
        }

        const float det = sIxx*sIyy - sIxy*sIxy;
        const float trace = sIxx+sIyy;
        cv::Vec2f uv;
        const double r_score = trace*trace/det;
        if (det == 0.0 || r_score > r_thresh_)
        {
          uv[0] = 0.0;
          uv[1] = 0.0;
        }
        else
        {
          uv[0] = (-sIyy*sIxt + sIxy*sIyt)/det;
          uv[1] = (sIxy*sIxt - sIxx*sIyt)/det;
        }
        flow_u.at<float>(r,c) = uv[0];
        flow_v.at<float>(r,c) = uv[1];
      }
    }
    std::vector<cv::Mat> outs;
    outs.push_back(flow_u);
    outs.push_back(flow_v);
    return outs;
  }

  cv::Mat reduce(cv::Mat& input)
  {
    cv::Mat output;// = input.clone();
    cv::pyrDown(input, output);
    return output;
  }

  cv::Mat expand(cv::Mat& input)
  {
    cv::Mat output(input.rows*2, input.cols*2, CV_32FC1);
    cv::pyrUp(input, output, output.size());
    return output;
  }

  cv::Mat smooth(cv::Mat& input, int n=1)
  {
    cv::Mat sm = input.clone();
#ifdef MEDIAN_FILTER_FLOW
    cv::medianBlur(input, sm, 3);
#else // MEDIAN_FILTER_FLOW
    for (int l = 0; l < n; ++l)
    {
      sm = reduce(sm);
    }
    for (int l = 0; l < n; ++l)
    {
      sm = expand(sm);
    }
#endif // MEDIAN_FILTER_FLOW
    return sm;
  }

  cv::Mat warp(cv::Mat& i2, cv::Mat& vx, cv::Mat& vy)
  {
    cv::Mat warpI2(i2.rows, i2.cols, i2.type(), cv::Scalar(0.0));
    // cv::Mat warpI3(i2.rows, i2.cols, i2.type(), cv::Scalar(0.0));
    cv::Mat map_x(vx.rows, vx.cols, CV_32FC1);
    cv::Mat map_y(vy.rows, vy.cols, CV_32FC1);

    for (int y = 0; y < map_x.rows; ++y)
    {
      for (int x = 0; x < map_x.cols; ++x)
      {
        map_x.at<float>(y,x) = x + vx.at<float>(y,x);
        map_y.at<float>(y,x) = y + vy.at<float>(y,x);
      }
    }

    // cv::remap(i2, warpI3, map_x, map_y, cv::INTER_NEAREST);
    cv::remap(i2, warpI2, map_x, map_y, cv::INTER_LINEAR);

    // for (int y = 0; y < i2.rows; ++y)
    // {
    //   for (int x = 0; x < i2.cols; ++x)
    //   {
    //     if (isnan(warpI2.at<float>(y,x)))
    //     {
    //       warpI2.at<float>(y,x) = warpI3.at<float>(y,x);
    //     }
    //   }
    // }
    return warpI2;
  }

  //
  // Getters and Setters
  //
  void setWinSize(int win_size)
  {
    win_size_ = win_size;
  }

  void setNumLevels(int num_levels)
  {
    max_level_ = num_levels-1;
  }

  double r_thresh_;
 protected:
  int win_size_;
  int max_level_;
  cv::Mat dx_kernel_;
  cv::Mat dy_kernel_;
  cv::Mat g_kernel_;
  cv::Mat optic_g_kernel_;
};

class FeatureTracker
{
 public:
  FeatureTracker(std::string name, double hessian_thresh=250, int num_octaves=4,
                 int num_layers=2, bool extended=true, bool upright=false) :
      surf_(hessian_thresh, num_octaves, num_layers, extended, upright),
      initialized_(false), ratio_threshold_(0.5), window_name_(name),
      min_flow_thresh_(0), max_corners_(500), klt_corner_thresh_(0.3),
      klt_corner_min_dist_(2)
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
    updateCurrentDescriptors(frame, cv::Mat());
    prev_keypoints_ = cur_keypoints_;
    prev_descriptors_ = cur_descriptors_;
    initialized_ = true;
  }

  AffineFlowMeasures updateTracksKLT(cv::Mat& cur_frame, cv::Mat& prev_frame)
  {
    AffineFlowMeasures sparse_flow;
    std::vector<cv::Point2f> prev_points;
    std::vector<cv::Point2f> new_points;
    ROS_INFO_STREAM("max_corners: " << max_corners_);
    ROS_INFO_STREAM("quality_level: " << klt_corner_thresh_);
    ROS_INFO_STREAM("min_distance: " << klt_corner_min_dist_);
    cv::goodFeaturesToTrack(prev_frame, prev_points, max_corners_,
                            klt_corner_thresh_, klt_corner_min_dist_);
    ROS_INFO_STREAM("Found " << prev_points.size() << " corners.");
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_frame, cur_frame, prev_points, new_points,
                             status, err);
    int moving_points = 0;
    for (unsigned int i = 0; i < prev_points.size(); i++)
    {
      if (! status[i]) continue;
      int dx = prev_points[i].x - new_points[i].x;
      int dy = prev_points[i].y - new_points[i].y;
      sparse_flow.push_back(AffineFlowMeasure(new_points[i].x, new_points[i].y,
                                              dx, dy));
      if (abs(sparse_flow[i].u) + abs(sparse_flow[i].v) > min_flow_thresh_)
        moving_points++;
    }
    ROS_INFO_STREAM(window_name_ << ": num moving points: " << moving_points);

#ifdef DISPLAY_TRACKER_OUTPUT
    cv::Mat display_cur_frame(cur_frame.rows, cur_frame.cols, CV_8UC3);;
    cv::cvtColor(cur_frame, display_cur_frame, CV_GRAY2BGR);
    for (unsigned int i = 0; i < sparse_flow.size(); i++)
    {
      if (abs(sparse_flow[i].u) + abs(sparse_flow[i].v) > min_flow_thresh_)
      {
        ROS_DEBUG_STREAM("Point is moving (" << sparse_flow[i].u << ", "
                         << sparse_flow[i].v << ")");
        cv::line(display_cur_frame,
                 cv::Point(sparse_flow[i].x, sparse_flow[i].y),
                 cv::Point(sparse_flow[i].x + sparse_flow[i].u,
                           sparse_flow[i].y + sparse_flow[i].v),
                 cv::Scalar(0,0,255), 1);
      }
    }

    cv::imshow(window_name_, display_cur_frame);
#endif // DISPLAY_TRACKER_OUTPUT
    return sparse_flow;
  }

  AffineFlowMeasures updateTracks(const cv::Mat& frame)
  {
    return updateTracks(frame, cv::Mat());
  }

  AffineFlowMeasures updateTracks(const cv::Mat& frame, const cv::Mat& mask)
  {
    cur_keypoints_.clear();
    cur_descriptors_.clear();
    updateCurrentDescriptors(frame, mask);

    std::vector<int> matches_cur;
    std::vector<int> matches_prev;
    AffineFlowMeasures sparse_flow;
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
      sparse_flow.push_back(AffineFlowMeasure(
          cur_keypoints_[matches_cur[i]].pt.x,
          cur_keypoints_[matches_cur[i]].pt.y, dx, dy));
      if (abs(sparse_flow[i].u) + abs(sparse_flow[i].v) > min_flow_thresh_)
        moving_points++;
    }
    ROS_DEBUG_STREAM(window_name_ << ": num moving points: " << moving_points);

#ifdef DISPLAY_TRACKER_OUTPUT
    cv::Mat display_frame(frame.rows, frame.cols, CV_8UC3);;
    cv::cvtColor(frame, display_frame, CV_GRAY2BGR);
    for (unsigned int i = 0; i < matches_cur.size(); i++)
    {
      if (abs(sparse_flow[i].u) + abs(sparse_flow[i].v) > min_flow_thresh_)
      {
        ROS_DEBUG_STREAM("Point is moving (" << sparse_flow[i].u << ", "
                         << sparse_flow[i].v << ")");
        cv::line(display_frame,
                 prev_keypoints_[matches_prev[i]].pt,
                 cur_keypoints_[matches_cur[i]].pt,
                 cv::Scalar(0,0,255), 1);
      }
    }

    cv::imshow(window_name_, display_frame);
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

  void updateCurrentDescriptors(const cv::Mat& frame, const cv::Mat& mask)
  {
    std::vector<float> raw_descriptors;
    try
    {
      // if (use_fast_)
      // {
      //   cv::FAST(frame, cur_keypoints_, 9, true);
      //   // TODO: Remove keypoints outside the mask
      //   surf_(frame, mask, cur_keypoints_, raw_descriptors, true);
      // }
      // else
      // {
      surf_(frame, mask, cur_keypoints_, raw_descriptors, false);
      // }
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

  void setKLTCornerThresh(double corner_thresh)
  {
    klt_corner_thresh_ = corner_thresh;
  }

  void setKLTCornerMinDist(double min_dist)
  {
    klt_corner_min_dist_ = min_dist;
  }

  void setKLTMaxCorners(int max_corners)
  {
    max_corners_ = max_corners;
  }

  void stop()
  {
    initialized_ = false;
  }
 public:
  cv::SURF surf_;
 protected:
  std::vector<cv::KeyPoint> prev_keypoints_;
  std::vector<cv::KeyPoint> cur_keypoints_;
  std::vector<Descriptor> prev_descriptors_;
  std::vector<Descriptor> cur_descriptors_;
  bool initialized_;
  double ratio_threshold_;
  std::string window_name_;
  int min_flow_thresh_;
  int max_corners_;
  double klt_corner_thresh_;
  double klt_corner_min_dist_;
};

class MotionGraphcut
{
 public:
  MotionGraphcut(double workspace_background_weight = 1.0f,
                 double min_weight = 0.01, double magnitude_thresh=0.1,
                 double flow_gain = 0.3, int arm_grow_radius=2) :
      workspace_background_weight_(workspace_background_weight),
      min_weight_(min_weight), magnitude_thresh_(magnitude_thresh),
      flow_gain_(flow_gain), arm_grow_radius_(arm_grow_radius)
  {
  }

  virtual ~MotionGraphcut()
  {
  }

  /**
   * Segment moving stuff from static stuff using graphcut
   *
   * @param color_frame    The current color image to segment
   * @param depth_frame    The current depth image to segment
   * @param u              Flow dx/dt
   * @param v              Flow dy/dt
   * @param eigen_scores    Scores corresponding to texture at the point
   * @param workspace_mask Binary image where white is valid locations for things
   *                       to be moving at
   * @param arm_locs Image locations of projected arm kinematics
   *
   * @return A binary image where white is the foreground (moving) regions and
   *         black is the background (static) regions
   */
  cv::Mat operator()(cv::Mat& color_frame, cv::Mat& depth_frame,
                     cv::Mat& u, cv::Mat& v, cv::Mat& workspace_mask,
                     cv::Mat& table_heights, ArmModel arm_locs)
  {
    const int R = color_frame.rows;
    const int C = color_frame.cols;
    int num_nodes = R*C;
    int num_edges = ((C-1)*3+1)*(R-1)+(C-1);
    GraphType *g;
    g = new GraphType(num_nodes, num_edges);

#ifdef VISUALIZE_GRAPH_WEIGHTS
    cv::Mat fg_weights(color_frame.size(), CV_32FC1);
    cv::Mat bg_weights(color_frame.size(), CV_32FC1);
    cv::Mat table_weights(color_frame.size(), CV_32FC1);
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
    cv::Mat left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS

    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        g->add_node();
        float magnitude = std::sqrt(u.at<float>(r,c)*u.at<float>(r,c) +
                                    v.at<float>(r,c)*v.at<float>(r,c));
        // Check if we are hardcoding this spot to background
        if (workspace_mask.at<uchar>(r,c) == 0)
        {
          g->add_tweights(r*C+c, min_weight_, workspace_background_weight_);

#ifdef VISUALIZE_GRAPH_WEIGHTS
          fg_weights.at<float>(r,c) = min_weight_;
          bg_weights.at<float>(r,c) = workspace_background_weight_;
          table_weights.at<float>(r,c) = min_weight_;
#endif // VISUALIZE_GRAPH_WEIGHTS
          continue;
        }
        const float mag_score = max(getFlowFGScore(magnitude), min_weight_);
        const float table_score = max(getTableScore(color_frame.at<cv::Vec3f>(r,c),
            fabs(table_heights.at<float>(r,c))), min_weight_);
        const float not_mag_score = max(1.0 - mag_score, min_weight_);
        const float bg_score = not_mag_score + table_score;
        g->add_tweights(r*C+c, mag_score, bg_score);
#ifdef VISUALIZE_GRAPH_WEIGHTS
        fg_weights.at<float>(r,c) = mag_score;
        bg_weights.at<float>(r,c) = bg_score;
        table_weights.at<float>(r,c) = table_score;
#endif // VISUALIZE_GRAPH_WEIGHTS
      }
    }
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        // Connect node to previous ones
        if (c > 0)
        {
          // Add left-link
          float w_l = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r,c-1),
                                    depth_frame.at<float>(r,c-1));
          g->add_edge(r*C+c, r*C+c-1, /*capacities*/ w_l, w_l);
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
          left_weights.at<float>(r,c) = w_l;
#endif // VISUALIZE_EDGE_GRAPH_WEIGHTS
        }
        if (r > 0)
        {
          // Add up-link
          float w_u = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r-1,c),
                                    depth_frame.at<float>(r-1,c));
          g->add_edge(r*C+c, (r-1)*C+c, /*capacities*/ w_u, w_u);
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
          up_weights.at<float>(r,c) = w_u;
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS
          // Add up-left-link
          if (c > 0)
          {
            float w_ul = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                       depth_frame.at<float>(r,c),
                                       color_frame.at<cv::Vec3f>(r-1,c-1),
                                       depth_frame.at<float>(r-1,c-1));
            g->add_edge(r*C+c, (r-1)*C+c-1, /*capacities*/ w_ul, w_ul);
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
          up_left_weights.at<float>(r,c) = w_ul;
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS
          }
        }
      }
    }

#ifdef VISUALIZE_GRAPH_WEIGHTS
    cv::imshow("fg_weights", fg_weights);
    cv::imshow("bg_weights", bg_weights);
    // double table_max=0;
    // cv::minMaxLoc(table_weights, NULL, &table_max);
    // table_weights /= table_max;
    cv::imshow("table_weights", table_weights);
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
    double up_max = 1.0;
    cv::minMaxLoc(up_weights, NULL, &up_max);
    up_weights /= up_max;

    cv::imshow("up_weights", up_weights);
    double left_max = 1.0;
    cv::minMaxLoc(left_weights, NULL, &left_max);
    left_weights /= left_max;
    cv::imshow("left_weights", left_weights);
    double up_left_max = 1.0;
    cv::minMaxLoc(up_left_weights, NULL, &up_left_max);
    up_left_weights /= up_max;
    cv::imshow("up_left_weights", up_left_weights);
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS

    // int flow = g->maxflow(false);
    g->maxflow(false);

    // Convert output into image
    cv::Mat segs = convertFlowResultsToCvMat(g, R, C);
    delete g;
    return segs;
  }

  /**
   * Method to segment the arm from the rest of the stuff moving in the scene
   *
   * @param color_frame    Current color frame
   * @param depth_frame    Current depth frame
   * @param moving_mask    Binary image representing where the moving stuff is
   * @param workspace_mask Binary image where white is valid locations for things
   *                       to be moving at
   * @param arms           Position of the arm projected into the image frame
   *
   * @return               Mask of the predicted arm in the image
   */
  cv::Mat segmentArmFromMoving(cv::Mat& color_frame_in, cv::Mat& depth_frame_in,
                               cv::Mat& workspace_mask_in, ArmModel& arms,
                               int min_arm_x, int max_arm_x, int min_arm_y,
                               int max_arm_y)

  {
    // NOTE: We examine only a subwindow in the image to avoid too make things
    // more efficient
    const int crop_min_x = max(0, min_arm_x - arm_search_radius_);
    const int crop_max_x = min(color_frame_in.cols,
                                    max_arm_x + arm_search_radius_);
    const int crop_min_y = max(0, min_arm_y - arm_search_radius_);
    const int crop_max_y = min(color_frame_in.rows,
                                    max_arm_y + arm_search_radius_);
    cv::Rect roi(crop_min_x, crop_min_y, crop_max_x-crop_min_x,
                 crop_max_y-crop_min_y);
    cv::Mat color_frame = color_frame_in(roi);
    cv::Mat depth_frame = depth_frame_in(roi);
    cv::Mat workspace_mask = workspace_mask_in(roi);

    const int R = color_frame.rows;
    const int C = color_frame.cols;

    int num_nodes = R*C;
    int num_edges = ((C-1)*3+1)*(R-1)+(C-1);
    GraphType *g;
    g = new GraphType(num_nodes, num_edges);

#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
    cv::Mat fg_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat bg_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat dist_img(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
    cv::Mat left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS

    // One gaussian estimated from wrist to elbow, elbow to forearm and a
    // separate one is estimated from the gripper tip to wrist
    if (arms[0].size() == 0)
    {
      ROS_WARN_STREAM("No hands!");
    }
    if (arms[1].size() == 0)
    {
      ROS_WARN_STREAM("No arms!");
    }
    std::vector<cv::Vec3f> hand_stats = getImagePointGaussian(color_frame,
                                                              arms[0],
                                                              crop_min_x,
                                                              crop_min_y);
    std::vector<cv::Vec3f> arm_stats = getImagePointGaussian(color_frame,
                                                             arms[1],
                                                             crop_min_x,
                                                             crop_min_y);
    // Tie weights to fg / bg
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        g->add_node();
#ifdef USE_WORKSPACE_MASK_FOR_ARM
        if (workspace_mask.at<uchar>(r,c) == 0)
        {
          g->add_tweights(r*C+c, min_weight_, workspace_background_weight_);
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
          fg_weights.at<float>(r,c) = min_weight_;
          bg_weights.at<float>(r,c) = workspace_background_weight_;
          dist_img.at<float>(r,c) = min_weight_;
#endif // VISUALIZE_GRAPH_WEIGHTS
          continue;
        }
#endif // USE_WORKSPACE_MASK_FOR_ARM
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
        const float me_score = max(getArmFGScore(color_frame, depth_frame, r, c,
                                                 arm_stats, hand_stats, arms,
                                                 roi, dist_img), min_weight_);
#else // VISUALIZE_ARM_GRAPH_WEIGHTS
        const float me_score = max(getArmFGScore(color_frame, depth_frame, r, c,
                                                 arm_stats, hand_stats, arms,
                                                 roi), min_weight_);
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
        const float not_me_score = max(1.0 - me_score, min_weight_);
        g->add_tweights(r*C+c, me_score, not_me_score);
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
        fg_weights.at<float>(r,c) = me_score;
        bg_weights.at<float>(r,c) = not_me_score;
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
      }
    }
    // Add edge weights
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        // Connect node to previous ones
        if (c > 0)
        {
          // Add left-link
          float w_l = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r,c-1),
                                    depth_frame.at<float>(r,c-1));
          g->add_edge(r*C+c, r*C+c-1, /*capacities*/ w_l, w_l);
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
          left_weights.at<float>(r,c) = w_l;
#endif // VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
        }
        if (r > 0)
        {
          // Add up-link
          float w_u = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r-1,c),
                                    depth_frame.at<float>(r-1,c));
          g->add_edge(r*C+c, (r-1)*C+c, /*capacities*/ w_u, w_u);
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
          up_weights.at<float>(r,c) = w_u;
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS

          // Add up-left-link
          if (c > 0)
          {
            float w_ul = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                       depth_frame.at<float>(r,c),
                                       color_frame.at<cv::Vec3f>(r-1,c-1),
                                       depth_frame.at<float>(r-1,c-1));
            g->add_edge(r*C+c, (r-1)*C+c-1, /*capacities*/ w_ul, w_ul);
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
            up_left_weights.at<float>(r,c) = w_ul;
#endif // VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS

          }
        }
      }
    }
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
    cv::imshow("fg_weights_arm", fg_weights);
    cv::imshow("bg_weights_arm", bg_weights);
    cv::imshow("arm_dist_scores", dist_img);
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
    double up_max = 1.0;
    cv::minMaxLoc(up_weights, NULL, &up_max);
    up_weights /= up_max;
    double left_max = 1.0;
    cv::minMaxLoc(left_weights, NULL, &left_max);
    left_weights /= left_max;
    double up_left_max = 1.0;
    cv::minMaxLoc(up_left_weights, NULL, &up_left_max);
    up_left_weights /= up_max;
    cv::imshow("up_weights_arm", up_weights);
    cv::imshow("left_weights_arm", left_weights);
    cv::imshow("up_left_weights_arm", up_left_weights);
#endif // VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS

    // int flow = g->maxflow(false);
    g->maxflow(false);

    // Convert output into image
    cv::Mat segs = convertFlowResultsToCvMat(g, R, C, roi,
                                             color_frame_in.size());
    delete g;
    return segs;
  }

  float getFlowFGScore(float magnitude)
  {
    return min(1.0, max(flow_gain_*exp(magnitude), 0.0));
  }

  float getTableScore(cv::Vec3f cur_c, float height_from_table)
  {
    const float dist_score = exp(-height_from_table/table_height_var_);
#ifndef USE_TABLE_COLOR_ESTIMATE
    return min(1.0, max(dist_score, 0.0));
#else // USE_TABLE_COLOR_ESTIMATE
    const float h_score = 1.0-fabs(cur_c[0] - table_stats_[0][0])/(
        table_stats_[1][0] + arm_color_var_add_);
    const float s_score = 1.0-fabs(cur_c[1] - table_stats_[0][1])/(
        table_stats_[1][1] + arm_color_var_add_);
    const float table_score = (h_score + s_score)/2.0+0.5*dist_score;
    return table_score;
#endif // USE_TABLE_COLOR_ESTIMATE
  }
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
  float getArmFGScore(cv::Mat& color_frame, cv::Mat& depth_frame, int r, int c,
                      std::vector<cv::Vec3f>& arm_stats,
                      std::vector<cv::Vec3f>& hand_stats, ArmModel& arms,
                      cv::Rect& roi, cv::Mat& dist_img)
#else // VISUALIZE_ARM_GRAPH_WEIGHTS
  float getArmFGScore(cv::Mat& color_frame, cv::Mat& depth_frame, int r, int c,
                      std::vector<cv::Vec3f>& arm_stats,
                      std::vector<cv::Vec3f>& hand_stats, ArmModel& arms,
                      cv::Rect& roi)
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
  {
    const float dist_score = exp(-arms.distanceToArm(
        cv::Point2f(c+roi.x,r+roi.y), depth_frame)/arm_dist_var_);
#ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
    dist_img.at<float>(r,c) = dist_score;
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS
    cv::Vec3f cur_c = color_frame.at<cv::Vec3f>(r,c);
    const float arm_h_score = 1.0-fabs(cur_c[0] - arm_stats[0][0])/(
        arm_stats[1][0] + arm_color_var_add_);
    const float arm_s_score = 1.0-fabs(cur_c[1] - arm_stats[0][1])/(
        arm_stats[1][1] + arm_color_var_add_);
    const float arm_v_score = 1.0-fabs(cur_c[2] - arm_stats[0][2])/(
        arm_stats[1][2] + arm_color_var_add_);
    const float arm_score = (arm_alpha_*(arm_h_score + arm_s_score +
                                         arm_v_score)/3.0 +
                             arm_beta_*dist_score);
    const float hand_h_score = 1.0-fabs(cur_c[0] - hand_stats[0][0])/(
        hand_stats[1][0] + arm_color_var_add_);
    const float hand_s_score = 1.0-fabs(cur_c[1] - hand_stats[0][1])/(
        hand_stats[1][1] + arm_color_var_add_);
    const float hand_v_score = 1.0-fabs(cur_c[2] - hand_stats[0][2])/(
        hand_stats[1][2] + arm_color_var_add_);
    const float hand_score = (arm_alpha_*(hand_h_score + hand_s_score +
                                          hand_v_score) / 3.0 +
                              arm_beta_*dist_score);
    return max(hand_score, arm_score);
  }

  std::vector<cv::Vec3f> getImagePointGaussian(cv::Mat& color_frame,
                                               std::vector<cv::Point> points,
                                               int min_x=0, int min_y=0)
  {
    // Calculate color means and variances
    const int C = color_frame.cols;
    const int R = color_frame.rows;
    int pixel_count = 0;
    cv::Vec3f means;
    means[0] = 0.0;
    means[1] = 0.0;
    means[2] = 0.0;
    for (unsigned int i = 0; i < points.size(); ++i)
    {
      for (int r = max(0, points[i].y - min_y - arm_grow_radius_);
           r < min(points[i].y - min_y + arm_grow_radius_, R); ++r)
      {
        for (int c = max(0, points[i].x - min_x - arm_grow_radius_);
             c < min(points[i].x - min_x + arm_grow_radius_, C); ++c)
        {
          cv::Vec3f cur_color = color_frame.at<cv::Vec3f>(r,c);
          means += cur_color;
          ++pixel_count;
        }
      }
    }
    if (pixel_count > 0)
    {
      means[0] /= pixel_count;
      means[1] /= pixel_count;
      means[2] /= pixel_count;
    }
    else
    {
      ROS_WARN_STREAM("Calculating stats for 0 pixels");
    }
    cv::Vec3f vars;
    vars[0] = 0.0;
    vars[1] = 0.0;
    vars[2] = 0.0;
    for (unsigned int i = 0; i < points.size(); ++i)
    {
      for (int r = max(0, points[i].y - min_y -arm_grow_radius_);
           r < min(points[i].y - min_y + arm_grow_radius_, R); ++r)
      {
        for (int c = max(0, points[i].x - min_x - arm_grow_radius_);
             c < min(points[i].x - min_x + arm_grow_radius_, C); ++c)
        {
          cv::Vec3f diff = color_frame.at<cv::Vec3f>(r,c);
          diff = diff.mul(diff);
          vars += diff;
        }
      }
    }
    vars[0] /=  (pixel_count+1.0);
    vars[1] /=  (pixel_count+1.0);
    vars[2] /=  (pixel_count+1.0);
    std::vector<cv::Vec3f> stats;
    stats.push_back(means);
    stats.push_back(vars);
    return stats;
  }

  void setTableColorStats(cv::Mat& color_frame, std::vector<cv::Point>& pts)
  {
    cv::Mat color_frame_hsv(color_frame.size(), color_frame.type());
    cv::cvtColor(color_frame, color_frame_hsv, CV_BGR2HSV);
    cv::Mat color_frame_f(color_frame_hsv.size(), CV_32FC3);
    color_frame_hsv.convertTo(color_frame_f, CV_32FC3, 1.0/255, 0);
    table_stats_ = getImagePointGaussian(color_frame_f, pts);
  }

  cv::Mat convertFlowResultsToCvMat(GraphType *g, int R, int C)
  {
    cv::Mat segs(R, C, CV_8UC1, cv::Scalar(0));
    for (int r = 0; r < R; ++r)
    {
      uchar* seg_row = segs.ptr<uchar>(r);
      for (int c = 0; c < C; ++c)
      {
        int label = (g->what_segment(r*C+c) == GraphType::SOURCE);
        seg_row[c] = label*255;
      }
    }
    return segs;
  }

  cv::Mat convertFlowResultsToCvMat(GraphType *g, int R, int C,
                                    cv::Rect roi, cv::Size out_size)
  {
    cv::Mat segs(out_size, CV_8UC1, cv::Scalar(0));
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        int label = (g->what_segment(r*C+c) == GraphType::SOURCE);
        segs.at<uchar>(r+roi.y, c+roi.x) = label*255;
      }
    }
    return segs;
  }

  float getEdgeWeight(cv::Vec3f c0, float d0, cv::Vec3f c1, float d1)
  {
    cv::Vec3f c_d = c0-c1;
    float w_d = d0-d1;
    float w_c = w_c_alpha_*exp(fabs(c_d[0])) + w_c_beta_*exp(fabs(c_d[1])) +
        /*w_c_beta_*exp(fabs(c_d[2])) +*/ w_c_gamma_*exp(fabs(w_d));
    return w_c;
  }

  enum SegLabels
  {
    MOVING,
    ARM,
    TABLE,
    BG
  };

  double workspace_background_weight_;
  double min_weight_;
  double w_c_alpha_;
  double w_c_beta_;
  double w_c_gamma_;
  double magnitude_thresh_;
  double flow_gain_;
  double table_height_var_;
  double arm_dist_var_;
  double arm_color_var_add_;
  double arm_alpha_;
  double arm_beta_;
  int arm_grow_radius_;
  int arm_search_radius_;
  std::vector<cv::Vec3f> table_stats_;
};

class ObjectSingulation
{
 public:
  ObjectSingulation(int kmeans_max_iter=200, double kmeans_epsilon=0.5,
                    int kmeans_tries=5, int affine_estimate_radius=5,
                    double surf_hessian=100) :
      kmeans_max_iter_(kmeans_max_iter), kmeans_epsilon_(kmeans_epsilon),
      kmeans_tries_(kmeans_tries),
      affine_estimate_radius_(affine_estimate_radius),
      ft_("obj singulation", surf_hessian)
  {
  }

  PoseStamped getPushVector(cv::Mat& motion_mask, cv::Mat& arm_mask,
                            cv::Mat& color_img, cv::Mat& depth_img,
                            cv::Mat& u, cv::Mat& v)
  {
    PoseStamped push_dir;
    // cv::Mat boundary_img = getObjectBoundaryStrengths(motion_mask, arm_mask,
    //                                                   color_img, depth_img);
    cv::Mat stuff_mask = motion_mask-arm_mask;
    // Get flow clusters
    AffineFlowMeasures centers = clusterFlowFields(color_img, depth_img,
                                                   u, v, stuff_mask);
    // TODO: Determine push_direction based on centers and their directions
    return push_dir;
  }

  Pose2D determinePushVector(std::vector<cv::Vec2f> centers)
  {
    Pose2D push_pose;
    return push_pose;
  }

  cv::Mat getObjectBoundaryStrengths(cv::Mat& motion_mask, cv::Mat& arm_mask,
                                     cv::Mat& color_img, cv::Mat& depth_img)
  {
    cv::Mat boundary_img(color_img.size(), CV_32FC1);
    return boundary_img;
  }

  AffineFlowMeasures clusterFlowFields(cv::Mat& color_img, cv::Mat& depth_img,
                                       cv::Mat& u, cv::Mat& v, cv::Mat& mask)
  {
    // return clusterFlowFieldsKMeans(color_img, depth_img, u, v, mask);
    return clusterSparseFlowKMeans(color_img, depth_img, u, v, mask);
  }

  AffineFlowMeasures clusterFlowFieldsKMeans(cv::Mat& color_img,
                                             cv::Mat& depth_img,
                                             cv::Mat& u, cv::Mat& v,
                                             cv::Mat& mask)
  {
    // Setup the samples as the flow vectors for the segmented moving region
    AffineFlowMeasures points;
    points.clear();
    for (int r = 0; r < mask.rows; ++r)
    {
      uchar* mask_row = mask.ptr<uchar>(r);
      for (int c = 0; c < mask.cols; ++c)
      {
        if ( mask_row[c] != 0)
        {
          AffineFlowMeasure p;
          p.u = u.at<float>(r,c);
          p.v = v.at<float>(r,c);
          p.x = c;
          p.y = r;
          p.a = estimateAffineTransform(u, v, r, c, affine_estimate_radius_);
          if (std::sqrt(p.u*p.u+p.v*p.v) > 1.0 )
            points.push_back(p);
        }
      }
    }

    if (points.size() < 1)
    {
      AffineFlowMeasures cluster_centers;
      cluster_centers.clear();
      return cluster_centers;
    }
    AffineFlowMeasures cluster_centers = clusterAffineKMeans(color_img, u, v,
                                                             points);
    return cluster_centers;
  }

  AffineFlowMeasures clusterFlowFieldsRANSAC(cv::Mat& color_img,
                                             cv::Mat& depth_img,
                                             cv::Mat& u, cv::Mat& v,
                                             cv::Mat& mask)
  {
    AffineFlowMeasures points;
    for (int r = 0; r < mask.rows; ++r)
    {
      uchar* mask_row = mask.ptr<uchar>(r);
      for (int c = 0; c < mask.cols; ++c)
      {
        if ( mask_row[c] != 0)
        {
          AffineFlowMeasure p;
          p.u = u.at<float>(r,c);
          p.v = v.at<float>(r,c);
          p.x = c;
          p.y = r;
          if (std::sqrt(p.u*p.u+p.v*p.v) > 1.0 )
            points.push_back(p);
        }
      }
    }

    AffineFlowMeasures cluster_centers;
    if (points.size() < 1)
    {
      cluster_centers.clear();
      return cluster_centers;
    }

    // Perform RANSAC itteratively on the affine estimates to cluster a set of
    // affine movement regions
    int k = 0;
    AffineFlowMeasures active_points = points;
    while (active_points.size() > min_affine_point_set_size_ &&
           k < max_k_)
    {
      AffineFlowMeasures inliers;
      inliers.clear();
      cv::Mat new_estimate = affineRANSAC(active_points, inliers,
                                          ransac_inlier_percent_est_,
                                          ransac_epsilon_, max_ransac_iters_);
      AffineFlowMeasure new_center;
      new_center.a = new_estimate;
      new_center.label = k;
      new_center.x = 0;
      new_center.y = 0;
      for (unsigned int i = 0; i < inliers.size(); ++i)
      {
        new_center.x += inliers[i].x;
        new_center.y += inliers[i].y;

        // Set labels for the removed points
        for (unsigned int j = 0; j < points.size(); ++j)
        {
          if (points[j] == inliers[i])
          {
            points[j].label = k;
          }
        }
        // Remove inliers from active points
        for (unsigned int j = 0; j < active_points.size(); )
        {
          if (inliers[i] == active_points[j])
          {
            active_points.erase(active_points.begin()+j);
          }
          else
          {
            ++j;
          }
        }
      }
      if (inliers.size() > 0)
      {
        new_center.x /= inliers.size();
        new_center.y /= inliers.size();
        cv::Mat V = new_center.a*new_center.X();
        new_center.u = V.at<float>(0,0);
        new_center.v = V.at<float>(1,0);
      }
      else
      {
        new_center.x = 0;
        new_center.y = 0;
        new_center.u = 0;
        new_center.v = 0;
      }
      cluster_centers.push_back(new_center);
      ROS_INFO_STREAM("Fit affine transform " << k << " with center ("
                      << new_center.x << ", " << new_center.y << ")");
      ROS_INFO_STREAM("Number of points remaining: " << active_points.size());
      ++k;
    }

#ifdef DISPLAY_FLOW_FIELD_CLUSTERING
    // ROS_INFO_STREAM("Displaying clusters");
    displayClusterCenters(cluster_centers, points, color_img);
    // ROS_INFO_STREAM("Displayed clusters");
#endif // DISPLAY_FLOW_FIELD_CLUSTERING

    return cluster_centers;
  }

  AffineFlowMeasures clusterSparseFlowKMeans(cv::Mat& color_img,
                                             cv::Mat& depth_img,
                                             cv::Mat& u, cv::Mat& v,
                                             cv::Mat& mask)
  {
    cv::Mat img_bw;
    cv::cvtColor(color_img, img_bw, CV_BGR2GRAY);
    AffineFlowMeasures sparse_flow = ft_.updateTracks(img_bw, mask);
    // TODO: Try add in geometric matching here to complement the sparse feature
    // tracks
    for (unsigned int i = 0; i < sparse_flow.size(); ++i)
    {
      sparse_flow[i].a = estimateAffineTransform(u, v, sparse_flow[i].y,
                                                 sparse_flow[i].x,
                                                 affine_estimate_radius_);
    }
    AffineFlowMeasures cluster_centers;
    if (sparse_flow.size() < /*1*/ 2)
    {
      cluster_centers.clear();
      return cluster_centers;
    }
    cluster_centers = clusterAffineKMeans(color_img, u, v, sparse_flow);
    return cluster_centers;
  }

  AffineFlowMeasures clusterSparseFlowRANSAC(cv::Mat& color_img,
                                             cv::Mat& depth_img,
                                             cv::Mat& u, cv::Mat& v,
                                             cv::Mat& mask)
  {
    return clusterSparseFlowKMeans(color_img, depth_img, u, v, mask);
  }

  //
  // Core functions
  //

  AffineFlowMeasures clusterAffineKMeans(cv::Mat& color_img, cv::Mat& u,
                                         cv::Mat& v, AffineFlowMeasures& points)
  {
    const int num_samples = points.size();
#ifdef SCALE_AFFINE_DISTANCES
    const int r_scale = color_img.cols / 2;
#endif // SCALE_AFFINE_DISTANCES

#ifdef CLUSTER_AFFINE_WITH_IMAGE_LOCS
    const int num_sample_elements = 8;
#else // CLUSTER_AFFINE_WITH_IMAGE_LOCS
    const int num_sample_elements = 6;
#endif // CLUSTER_AFFINE_WITH_IMAGE_LOCS

    // Setup sample matrix for kmeans
    cv::Mat samples(num_samples, num_sample_elements, CV_32FC1);
    for (unsigned int i = 0; i < points.size(); ++i)
    {
      AffineFlowMeasure p = points[i];
      // TODO: This could be done better by reshaping p.a and setting it to a
      // submatrix of samples
#ifdef SCALE_AFFINE_DISTANCES
      samples.at<float>(i, 0) = p.a.at<float>(0,0)*r_scale;
      samples.at<float>(i, 1) = p.a.at<float>(0,1)*r_scale;
      samples.at<float>(i, 2) = p.a.at<float>(0,2);
      samples.at<float>(i, 3) = p.a.at<float>(1,0)*r_scale;
      samples.at<float>(i, 4) = p.a.at<float>(1,1)*r_scale;
      samples.at<float>(i, 5) = p.a.at<float>(1,2);
#else // SCALE_AFFINE_DISTANCES
      samples.at<float>(i, 0) = p.a.at<float>(0,0);
      samples.at<float>(i, 1) = p.a.at<float>(0,1);
      samples.at<float>(i, 2) = p.a.at<float>(0,2);
      samples.at<float>(i, 3) = p.a.at<float>(1,0);
      samples.at<float>(i, 4) = p.a.at<float>(1,1);
      samples.at<float>(i, 5) = p.a.at<float>(1,2);
#endif // SCALE_AFFINE_DISTANCES
#ifdef CLUSTER_AFFINE_WITH_IMAGE_LOCS
      samples.at<float>(i, 6) = p.x;
      samples.at<float>(i, 7) = p.y;
#endif // CLUSTER_AFFINE_WITH_IMAGE_LOCS
    }

    std::vector<cv::Mat> labels;
    std::vector<cv::Mat> centers;
    double compactness[max_k_];
    cv::TermCriteria term_crit(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                               kmeans_max_iter_, kmeans_epsilon_);

    AffineFlowMeasures cluster_centers;
    AffineFlowMeasures fewer_centers;
    AffineFlowMeasures best_centers;
    for (int K = 1; K <= min(max_k_, num_samples); ++K)
    {
      // Perform clustering with K centers
      cv::Mat labels_k;
      cv::Mat centers_k;
      double slack = cv::kmeans(samples, K, labels_k, term_crit,
                                kmeans_tries_, cv::KMEANS_PP_CENTERS,
                                centers_k);
      compactness[K-1] = slack;
      labels.push_back(labels_k);
      centers.push_back(centers_k);
      cluster_centers.clear();
      // Get descriptors for each cluster and compare them to the previous level
      for (int c = 0; c < K; ++c)
      {
        AffineFlowMeasure new_center;
        new_center.x = 0;
        new_center.y = 0;
        new_center.label = c;
        int num_members = 0;
        for (int i = 0; i < num_samples; ++i)
        {
          if (labels[K-1].at<uchar>(i,0) == c)
          {
            new_center.x += points[i].x;
            new_center.y += points[i].y;
            points[i].label = c;
            ++num_members;
          }
        }

        if (num_members <= 0 ||
            centers[K-1].cols == 0 || centers[K-1].rows == 0)
        {
          new_center.x = 0;
          new_center.y = 0;
          new_center.u = 0;
          new_center.v = 0;
        }
        else
        {
          new_center.x = new_center.x/num_members;
          new_center.y = new_center.y/num_members;

          // Correctly set the affine estimate
          // TODO: This could be done better by selecting a submatrix from centers
          // and then reshaping it
          new_center.a.create(2, 3, CV_32FC1);
          new_center.a.at<float>(0,0) = centers[K-1].at<float>(c,0);
          new_center.a.at<float>(0,1) = centers[K-1].at<float>(c,1);
          new_center.a.at<float>(0,2) = centers[K-1].at<float>(c,2);
          new_center.a.at<float>(1,0) = centers[K-1].at<float>(c,3);
          new_center.a.at<float>(1,1) = centers[K-1].at<float>(c,4);
          new_center.a.at<float>(1,2) = centers[K-1].at<float>(c,5);
#ifdef SCALE_AFFINE_DISTANCES
          new_center.a.at<float>(0,0) /= r_scale;
          new_center.a.at<float>(0,1) /= r_scale;
          new_center.a.at<float>(1,0) /= r_scale;
          new_center.a.at<float>(1,1) /= r_scale;
#endif // SCALE_AFFINE_DISTANCES

          // Estimate flow of the cluster center using affine transform estimate
          cv::Mat V = new_center.a*new_center.X();
          new_center.u = V.at<float>(0,0);
          new_center.v = V.at<float>(1,0);
        }
        cluster_centers.push_back(new_center);
      }
      // Compare current K centers to centers of cardinality K-1
      if (K > 1)
      {
        float farthest_nearest_neighbor = 0;
        for (unsigned int i = 0; i < cluster_centers.size(); ++i)
        {
          float nn = FLT_MAX;
          for (unsigned int j = 0; j < fewer_centers.size(); ++j)
          {
            float dist = cluster_centers[i] - fewer_centers[j];
            if (dist < nn)
              nn = dist;
          }
          if (nn > farthest_nearest_neighbor)
            farthest_nearest_neighbor = nn;
        }
        // If no new clusters have center far enough from the current clusters,
        // Then we choose the previous set as best
        if (farthest_nearest_neighbor < minimum_new_cluster_separation_)
        {
          best_centers = fewer_centers;
          break;
        }
      }
      // Store current estimates for comparing in next iteration of K
      fewer_centers = cluster_centers;
      best_centers = cluster_centers;
    }
    ROS_INFO_STREAM("Chose " << best_centers.size() << " clusters");
#ifdef DISPLAY_FLOW_FIELD_CLUSTERING
    displayClusterCenters(best_centers, points, color_img, 0);
#endif // DISPLAY_FLOW_FIELD_CLUSTERING
    return best_centers;
  }

  cv::Mat estimateAffineTransform(cv::Mat& u, cv::Mat& v,
                                  const int r, const int c, const int radius)
  {
    const int r_min = max(r - radius, 0);
    const int r_max = min(r + radius, u.rows-1);
    const int c_min = max(c - radius, 0);
    const int c_max = min(c + radius, u.cols-1);
    const int r_range = r_max-r_min+1;
    const int c_range = c_max-c_min+1;
    const int num_eqs = r_range*c_range*2;
    cv::Mat a(6, 1, CV_32FC1, cv::Scalar(1.0));
    if (num_eqs < 6)
    {
      ROS_WARN_STREAM("Too few equations; num equations is: " << num_eqs);
      cv::Mat A = a.reshape(1, 2);
      return A;
    }
    cv::Mat phi(num_eqs, 6, CV_32FC1, cv::Scalar(0.0));
    cv::Mat V(num_eqs, 1, CV_32FC1, cv::Scalar(0.0));
    for (int r = r_min, out_row = 0; r <= r_max; ++r)
    {
      for (int c = c_min; c <= c_max; ++c, ++out_row)
      {
        phi.at<float>(out_row, 0) = r;
        phi.at<float>(out_row, 1) = c;
        phi.at<float>(out_row, 2) = 1.0;
        V.at<float>(out_row, 0) = u.at<float>(r,c);
        ++out_row;
        phi.at<float>(out_row, 3) = r;
        phi.at<float>(out_row, 4) = c;
        phi.at<float>(out_row, 5) = 1.0;
        V.at<float>(out_row, 0) = v.at<float>(r,c);
      }
    }
    try
    {
      cv::solve(phi, V, a, cv::DECOMP_SVD);
    }
    catch (cv::Exception cve)
    {
      ROS_ERROR_STREAM(cve.what());
    }
    cv::Mat A = a.reshape(1, 2);
    return A;
  }

  cv::Mat estimateAffineTransform(AffineFlowMeasures& points)
  {
    const int num_eqs = points.size()*2;
    cv::Mat phi(num_eqs, 6, CV_32FC1, cv::Scalar(0.0));
    cv::Mat V(num_eqs, 1, CV_32FC1, cv::Scalar(0.0));
    cv::Mat a(6, 1, CV_32FC1, cv::Scalar(1.0));
    if (num_eqs < 6)
    {
      ROS_WARN_STREAM("Too few equations; num equations is: " << num_eqs);
      cv::Mat A = a.reshape(1, 2);
      return A;
    }
    for (unsigned int i = 0, out_row = 0; i < points.size(); ++i, ++out_row)
    {
      AffineFlowMeasure p = points[i];
      phi.at<float>(out_row, 0) = p.y;
      phi.at<float>(out_row, 1) = p.x;
      phi.at<float>(out_row, 2) = 1.0;
      V.at<float>(out_row, 0) = p.u;
      ++out_row;
      phi.at<float>(out_row, 3) = p.y;
      phi.at<float>(out_row, 4) = p.x;
      phi.at<float>(out_row, 5) = 1.0;
      V.at<float>(out_row, 0) = p.v;
    }
    cv::solve(phi, V, a, cv::DECOMP_SVD);
    cv::Mat A = a.reshape(1, 2);
    return A;
  }

  /**
   * See how closely the current affine estimate matches the estimated flow
   *
   * @param f The flow estimate
   * @param A The affine region estimate
   *
   * @return Distortion score SSD between V and V_hat
   */
  float getAffineDistortion(AffineFlowMeasure f, cv::Mat A)
  {
    cv::Mat X(3, 1, CV_32FC1, cv::Scalar(1.0));
    X.at<float>(0,0) = f.x;
    X.at<float>(1,0) = f.y;

    cv::Mat V(2, 1, CV_32FC1, cv::Scalar(1.0));
    V.at<float>(0,0) = f.u;
    V.at<float>(0,0) = f.v;

    cv::Mat d = V - A*X;
    return d.at<float>(0,0)*d.at<float>(0,0)+d.at<float>(1,0)*d.at<float>(1,0);
  }

  /**
   * Determine the set of inliers for the current affine estimate
   *
   * @param points The set of all points given to RANSAC
   * @param cur_transform The current transform estimate
   * @param epsilon The threshold for acceptance as an inlier
   *
   * @return The set of points which score less than epsilon
   */
  AffineFlowMeasures getAffineConsensus(AffineFlowMeasures points,
                                        cv::Mat cur_transform, float epsilon)
  {
    AffineFlowMeasures inliers;
    for (unsigned int i = 0; i < points.size(); ++i)
    {
      float score = getAffineDistortion(points[i], cur_transform);
      if (score < epsilon)
      {
        inliers.push_back(points[i]);
      }
    }
    return inliers;
  }

  /**
   * Method fits an affine motion estimate to a set of image points using RANSAC
   *
   * @param points The set of individual flow estimates
   * @param inliers Returns the set of points determined to be inliers
   * @param inlier_percent_est the estimated percent of inliers in points
   * @param epsilon the threshold for being an inlier
   * @param max_iterations The maximum number of sample iterations to execute
   * @param min_iterations The minimum number of sample iterations to execute
   *
   * @return The best fit affine estimate
   */
  cv::Mat affineRANSAC(AffineFlowMeasures& points, AffineFlowMeasures& inliers,
                       float inlier_percent_est, float epsilon,
                       int max_iterations=0, int min_iterations = 2)
  {
    AffineFlowMeasures sample_points;
    AffineFlowMeasures cur_inliers;
    AffineFlowMeasures best_inliers;
    best_inliers.clear();
    bool done;
    int iter = 0;

    // Compute max_iterations as function of inlier percetage
    if (max_iterations == 0)
    {
      // Percent certainty
      const float p = 0.99;
      // Number of model parameters
      const float s = 3.0f;
      max_iterations = log(1 - p) / log(1-pow(inlier_percent_est, s));
    }

    while ( !done )
    {
      // Randomly select 3 points
      sample_points.clear();
      for (int i = 0; i < 3; ++i)
      {
        int r_idx = (rand() % (points.size()+1));
        sample_points.push_back(points[r_idx]);
      }
      // Estimate affine flow from them
      cv::Mat cur_transform = estimateAffineTransform(sample_points);
      cur_inliers = getAffineConsensus(points, cur_transform, epsilon);
      // Update best estimate if we have more points
      if ( best_inliers.size() < cur_inliers.size() )
      {
        best_inliers = cur_inliers;
      }
      // Check if sampling should stop
      iter++;
      done = ((iter > min_iterations &&
               (static_cast<float>(best_inliers.size())/points.size() >
                inlier_percent_est ||
                best_inliers.size() > inlier_percent_est*points.size() )) ||
               iter > max_iterations);
    }
    inliers = best_inliers;
    return estimateAffineTransform(inliers);
  }

  //
  // Helper Functions
  //

    /**
   * Display the results for the segmentation.
   *
   * @param cluster_centers The estimated segmentation centers
   * @param samples All points used in clustering
   * @param color_img The color image associated with the estimate
   */
  void displayClusterCenters(AffineFlowMeasures& cluster_centers,
                             AffineFlowMeasures& samples,
                             cv::Mat& color_img, int cur_max_k=0)
  {
    cv::Mat flow_center_disp = color_img.clone();
    for (unsigned int i = 0; i < cluster_centers.size(); ++i)
    {
      cv::Point pt_a(cluster_centers[i].x, cluster_centers[i].y);
      cv::Point pt_b(pt_a.x - cluster_centers[i].u,
                     pt_a.y - cluster_centers[i].v);
      cv::circle(flow_center_disp, pt_a, 20, cv::Scalar(0,255,0));
      cv::line(flow_center_disp, pt_a, pt_b, cv::Scalar(0,255,0));
    }
    std::stringstream center_disp_name;
    center_disp_name << "Flow Cluster Centers";
    if (cur_max_k != 0) center_disp_name << " " << cur_max_k;
    cv::imshow(center_disp_name.str(), flow_center_disp);
    cv::Mat flow_clusters_disp = color_img.clone();
    std::vector<cv::Scalar> colors;
    for (unsigned int k = 0; k < cluster_centers.size(); ++k)
    {
      cv::Scalar rand_color;
      rand_color[0] = (static_cast<float>(rand()) /
                       static_cast<float>(RAND_MAX))*255;
      rand_color[1] = (static_cast<float>(rand()) /
                       static_cast<float>(RAND_MAX))*255;
      rand_color[2] = (static_cast<float>(rand()) /
                       static_cast<float>(RAND_MAX))*255;
      colors.push_back(rand_color);
    }
    for (unsigned int i = 0; i < samples.size(); ++i)
    {
      AffineFlowMeasure s = samples[i];
      if (s.label < 0 || s.label > (cluster_centers.size()-1)) continue;
      if (std::sqrt(s.u*s.u+s.v*s.v) > 1.0)
      {
        cv::Scalar cur_color = colors[s.label];
        cv::line(flow_clusters_disp, cv::Point(s.x, s.y),
                 cv::Point(s.x - s.u, s.y - s.v), cur_color);
      }
    }
    std::stringstream cluster_disp_name;
    cluster_disp_name << "Flow Clusters";
    if (cur_max_k != 0) cluster_disp_name << " " << cur_max_k;
    cv::imshow(cluster_disp_name.str(), flow_clusters_disp);
  }

  //
  // Class member variables
  //

  int kmeans_max_iter_;
  double kmeans_epsilon_;
  double ransac_epsilon_;
  double ransac_inlier_percent_est_;
  int kmeans_tries_;
  int max_k_;
  int affine_estimate_radius_;
  int min_affine_point_set_size_;
  int max_ransac_iters_;
  double minimum_new_cluster_separation_;
  FeatureTracker ft_;
};

class TabletopPushingPerceptionNode
{
 public:
  TabletopPushingPerceptionNode(ros::NodeHandle &n) :
      n_(n), n_private_("~"),
      image_sub_(n, "color_image_topic", 1),
      depth_sub_(n, "depth_image_topic", 1),
      cloud_sub_(n, "point_cloud_topic", 1),
      sync_(MySyncPolicy(15), image_sub_, depth_sub_, cloud_sub_),
      it_(n),
      track_server_(n, "seg_track_action",
                    boost::bind(&TabletopPushingPerceptionNode::trackerGoalCallback,
                                this, _1),
                    false),
      tf_(), have_depth_data_(false), tracking_(false),
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
    n_private_.param("below_table_z", below_table_z_, 0.1);
    n_private_.param("max_workspace_x", max_workspace_x_, 0.0);
    n_private_.param("max_workspace_y", max_workspace_y_, 0.0);
    n_private_.param("max_workspace_z", max_workspace_z_, 0.0);
    n_private_.param("min_pushing_x", min_pushing_x_, 0.0);
    n_private_.param("min_pushing_y", min_pushing_y_, 0.0);
    n_private_.param("max_pushing_x", max_pushing_x_, 0.0);
    n_private_.param("max_pushing_y", max_pushing_y_, 0.0);
    std::string default_workspace_frame = "/torso_lift_link";
    n_private_.param("workspace_frame", workspace_frame_,
                    default_workspace_frame);
    n_private_.param("min_table_z", min_table_z_, -0.5);
    n_private_.param("max_table_z", max_table_z_, 1.5);
    n_private_.param("autostart_tracking", tracking_, false);
    n_private_.param("num_downsamples", num_downsamples_, 2);
    std::string cam_info_topic_def = "/kinect_head/rgb/camera_info";
    n_private_.param("cam_info_topic", cam_info_topic_,
                    cam_info_topic_def);
    n_private_.param("table_ransac_thresh", table_ransac_thresh_, 0.01);

    base_output_path_ = "/home/thermans/sandbox/cut_out/";
    // Graphcut weights
    n_private_.param("mgc_workspace_bg_weight",
                    mgc_.workspace_background_weight_, 1.0);
    n_private_.param("mgc_min_weight", mgc_.min_weight_, 0.01);
    n_private_.param("mgc_w_c_alpha", mgc_.w_c_alpha_, 0.1);
    n_private_.param("mgc_w_c_beta",  mgc_.w_c_beta_, 0.1);
    n_private_.param("mgc_w_c_gamma", mgc_.w_c_gamma_, 0.1);
    n_private_.param("mgc_arm_grow_radius", mgc_.arm_grow_radius_, 2);
    n_private_.param("mgc_arm_search_radius", mgc_.arm_search_radius_, 50);
    // Lucas Kanade params
    n_private_.param("mgc_magnitude_thresh", mgc_.magnitude_thresh_, 0.1);
    n_private_.param("mgc_flow_gain", mgc_.flow_gain_, 0.3);
    n_private_.param("mgc_table_var", mgc_.table_height_var_, 0.03);
    n_private_.param("mgc_arm_dist_var", mgc_.arm_dist_var_, 20.0);
    n_private_.param("mgc_arm_color_var_add", mgc_.arm_color_var_add_, 0.1);
    n_private_.param("mgc_arm_color_weight", mgc_.arm_alpha_, 0.5);
    n_private_.param("mgc_arm_dist_weight", mgc_.arm_beta_, 0.5);
    int win_size = 5;
    n_private_.param("lk_win_size", win_size, 5);
    lkflow_.setWinSize(win_size);
    int num_levels = 4;
    n_private_.param("lk_num_levels", num_levels, 4);
    lkflow_.setNumLevels(num_levels);
    n_private_.param("lk_ratio_thresh", lkflow_.r_thresh_, 30.0);
    n_private_.param("max_flow_clusters", os_.max_k_, 2);
    n_private_.param("flow_cluster_max_iter", os_.kmeans_max_iter_, 200);
    n_private_.param("flow_cluster_epsilon", os_.kmeans_epsilon_, 0.05);
    n_private_.param("flow_cluster_attempts", os_.kmeans_tries_, 5);
    n_private_.param("affine_estimate_radius", os_.affine_estimate_radius_, 5);
    n_private_.param("min_affine_point_set_size", os_.min_affine_point_set_size_,
                     100);
    n_private_.param("max_ransac_iters", os_.max_ransac_iters_,
                     150);
    n_private_.param("affine_RANSAC_epsilon", os_.ransac_epsilon_, 1.0);
    n_private_.param("affine_RANSAC_inlier_percent",
                     os_.ransac_inlier_percent_est_, 0.05);
    n_private_.param("minimum_new_cluster_separation",
                     os_.minimum_new_cluster_separation_, 5.0);
    n_private_.param("surf_hessian_thresh", os_.ft_.surf_.hessianThreshold,
                     150.0);

    n_private_.param("image_hist_size", image_hist_size_, 5);

    // Setup ros node connections
    sync_.registerCallback(&TabletopPushingPerceptionNode::sensorCallback,
                           this);
    push_pose_server_ = n_.advertiseService(
        "get_push_pose", &TabletopPushingPerceptionNode::getPushPose, this);
    table_location_server_ = n_.advertiseService(
        "get_table_location", &TabletopPushingPerceptionNode::getTableLocation,
        this);
    motion_mask_pub_ = it_.advertise("motion_mask", 15);
    motion_img_pub_ = it_.advertise("motion_img", 15);
    arm_mask_pub_ = it_.advertise("arm_mask", 15);
    arm_img_pub_ = it_.advertise("arm_img", 15);
    track_server_.start();
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

    // Transform point cloud into the correct frame and convert to PCL struct
    XYZPointCloud cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    pcl_ros::transformPointCloud(workspace_frame_, cloud, cloud, tf_);

    // Convert nans to zeros
    for (int r = 0; r < depth_frame.rows; ++r)
    {
      float* depth_row = depth_frame.ptr<float>(r);
      for (int c = 0; c < depth_frame.cols; ++c)
      {
        float cur_d = depth_row[c];
        if (isnan(cur_d))
        {
          depth_row[c] = 0.0;
        }
      }
    }

    cv::Mat workspace_mask(color_frame.rows, color_frame.cols, CV_8UC1,
                           cv::Scalar(255));
    // Black out pixels in color and depth images outside of workspace
    // As well as outside of the crop window
    for (int r = 0; r < color_frame.rows; ++r)
    {
      uchar* workspace_row = workspace_mask.ptr<uchar>(r);
      for (int c = 0; c < color_frame.cols; ++c)
      {
        // NOTE: Cloud is accessed by at(column, row)
        pcl::PointXYZ cur_pt = cloud.at(c, r);
        if (cur_pt.x < min_workspace_x_ || cur_pt.x > max_workspace_x_ ||
            cur_pt.y < min_workspace_y_ || cur_pt.y > max_workspace_y_ ||
            cur_pt.z < min_workspace_z_ || cur_pt.z > max_workspace_z_ ||
            r < crop_min_y_ || c < crop_min_x_ || r > crop_max_y_ ||
            c > crop_max_x_ )
        {
          workspace_row[c] = 0;
        }
      }
    }

    // Downsample everything first
    cv::Mat color_frame_down = downSample(color_frame, num_downsamples_);
    cv::Mat depth_frame_down = downSample(depth_frame, num_downsamples_);
    cv::Mat workspace_mask_down = downSample(workspace_mask, num_downsamples_);

    // Save internally for use in the service callback
    prev_color_frame_ = cur_color_frame_.clone();
    prev_depth_frame_ = cur_depth_frame_.clone();
    prev_workspace_mask_ = cur_workspace_mask_.clone();
    prev_camera_header_ = cur_camera_header_;

    // Update the current versions
    cur_color_frame_ = color_frame_down.clone();
    cur_depth_frame_ = depth_frame_down.clone();
    cur_workspace_mask_ = workspace_mask_down.clone();
    cur_point_cloud_ = cloud;
    have_depth_data_ = true;
    cur_camera_header_ = img_msg->header;

    // Started via actionlib call
    cv::Mat seg_mask = segmentMovingStuff(cur_color_frame_, cur_depth_frame_,
                                          prev_color_frame_, prev_depth_frame_,
                                          cur_point_cloud_);
    prev_seg_mask_ = seg_mask.clone();
  }

  bool getTableLocation(LocateTable::Request& req, LocateTable::Response& res)
  {
    if ( have_depth_data_ )
    {
      res.table_centroid = getTablePlane(cur_point_cloud_);
      if ((res.table_centroid.pose.position.x == 0.0 &&
           res.table_centroid.pose.position.y == 0.0 &&
           res.table_centroid.pose.position.z == 0.0) ||
          res.table_centroid.pose.position.x < 0.0)
      {
        ROS_ERROR_STREAM("No plane found, leaving");
        res.found_table = false;
        return false;
      }
      res.found_table = true;
    }
    else
    {
      ROS_ERROR_STREAM("Calling getTableLocation prior to receiving sensor data.");
      res.found_table = false;
      return false;
    }
    return true;
  }

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    if ( have_depth_data_ )
    {
      res = findPushPose(cur_color_frame_, cur_depth_frame_, cur_point_cloud_,
                         req.use_guided);
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
                                  XYZPointCloud& cloud, bool use_guided)
  {
    PushPose::Response res;
    if (use_guided)
    {
      // TODO: Change this to guided based on proto object boundaries
      res.push_pose = os_.getPushVector(motion_mask_hist_.back(),
                                        arm_mask_hist_.back(),
                                        color_frame_hist_.back(),
                                        depth_frame_hist_.back(),
                                        flow_u_hist_.back(),
                                        flow_v_hist_.back());

    }
    else
    {
      res.push_pose = findRandomPushPose(cloud);
    }
    res.invalid_push_pose = false;
    return res;
  }

  //
  // Region tracking methods
  //

  void trackerGoalCallback(const tabletop_pushing::SegTrackGoalConstPtr &goal)
  {
    if (goal->init)
    {
      ROS_INFO_STREAM("Initializing tracker.");
      initTracker();
      tabletop_pushing::SegTrackResult result;
      track_server_.setSucceeded(result);
    }

    if (goal->start)
    {
      ROS_INFO_STREAM("Starting tracker.");
      startTracker();
      tabletop_pushing::SegTrackResult result;
      track_server_.setSucceeded(result);
    }
    else
    {
      ROS_INFO_STREAM("Stopping tracker.");
      stopTracker();
      tabletop_pushing::SegTrackResult result;
      if (goal->get_singulation_vector)
      {
        result.singulation_vector = os_.getPushVector(motion_mask_hist_.back(),
                                                      arm_mask_hist_.back(),
                                                      color_frame_hist_.back(),
                                                      depth_frame_hist_.back(),
                                                      flow_u_hist_.back(),
                                                      flow_v_hist_.back());
      }
      track_server_.setSucceeded(result);
      // track_server_.setPreempted();
    }
  }

  void initTracker()
  {
    tracker_initialized_ = false;
    tracking_ = false;
  }

  void startTracker()
  {
    // tracker_initialized_ = true;
    tracking_ = true;
  }

  void stopTracker()
  {
    // tracker_initialized_ = false;
    tracking_ = false;
  }

  //
  // Core method for calculation
  //
  cv::Mat segmentMovingStuff(cv::Mat& color_frame, cv::Mat& depth_frame,
                             cv::Mat& prev_color_frame, cv::Mat& prev_depth_frame,
                             XYZPointCloud& cloud)
  {
    if (!tracker_initialized_)
    {
      cam_info_ = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
          cam_info_topic_, n_, ros::Duration(5.0));

      table_centroid_ = getTablePlane(cloud);
      if (table_centroid_.pose.position.x == 0.0 &&
          table_centroid_.pose.position.y == 0.0 &&
          table_centroid_.pose.position.z == 0.0)
      {
        ROS_ERROR_STREAM("No plane found!");
      }
      else
      {
        min_workspace_z_ = table_centroid_.pose.position.z - below_table_z_;
        n_private_.setParam("min_workspace_z", min_workspace_z_);
        ROS_INFO_STREAM("Found plane");
      }

#ifdef AUTO_FLOW_CLUSTER
      cv::Mat bw_frame;
      cv::cvtColor(color_frame, bw_frame, CV_BGR2GRAY);
      os_.ft_.initTracks(bw_frame);
#endif // AUTO_FLOW_CLUSTER

      tracker_initialized_ = true;
      tracker_count_ = 0;
      cv::Mat empty_segments(color_frame.rows, color_frame.cols, CV_8UC1,
                             cv::Scalar(0));
      return empty_segments;
    }

    if (!tracking_)
    {
      cv::Mat empty_segments(color_frame.rows, color_frame.cols, CV_8UC1,
                             cv::Scalar(0));
      return empty_segments;
    }

    // Convert frame to floating point HSV
    // TODO: Consolidate into a single function call ?
    cv::Mat color_frame_hsv(color_frame.size(), color_frame.type());
    cv::cvtColor(color_frame, color_frame_hsv, CV_BGR2HSV);
    cv::Mat color_frame_f(color_frame_hsv.size(), CV_32FC3);
    color_frame_hsv.convertTo(color_frame_f, CV_32FC3, 1.0/255, 0);

    // Get optical flow
    std::vector<cv::Mat> flow_outs = lkflow_(color_frame, prev_color_frame);
    // Project locations of the arms and hands into the image
    int min_arm_x = 0;
    int max_arm_x = 0;
    int min_arm_y = 0;
    int max_arm_y = 0;
    ArmModel hands_and_arms = projectArmPoses(cur_camera_header_,
                                              color_frame.size(), min_arm_x,
                                              max_arm_x, min_arm_y, max_arm_y);
    // Get pixel heights above the table
    cv::Mat heights_above_table = getTableHeightDistances();
    // Perform graphcut for motion detection
    cv::Mat cut = mgc_(color_frame_f, depth_frame, flow_outs[0], flow_outs[1],
                       cur_workspace_mask_, heights_above_table,
                       hands_and_arms);
    // Perform graphcut for arm localization
    cv::Mat arm_cut(color_frame.size(), CV_8UC1, cv::Scalar(0));
    if (hands_and_arms[0].size() > 0 || hands_and_arms[1].size() > 0)
    {
      arm_cut = mgc_.segmentArmFromMoving(color_frame_f, depth_frame,
                                          cur_workspace_mask_, hands_and_arms,
                                          min_arm_x, max_arm_x, min_arm_y,
                                          max_arm_y);
    }

    // Publish the moving region stuff
    cv_bridge::CvImage motion_mask_msg;
    motion_mask_msg.image = cut;
    motion_mask_msg.header = cur_camera_header_;
    motion_mask_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    motion_mask_pub_.publish(motion_mask_msg.toImageMsg());

    // Publish arm stuff
    cv_bridge::CvImage arm_mask_msg;
    arm_mask_msg.image = arm_cut;
    arm_mask_msg.header = cur_camera_header_;
    arm_mask_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    arm_mask_pub_.publish(arm_mask_msg.toImageMsg());

    // Also publish color versions
    cv::Mat moving_regions_img;
    color_frame.copyTo(moving_regions_img, cut);
    cv_bridge::CvImage motion_img_msg;
    cv::Mat motion_img_send(cut.size(), CV_8UC3);
    moving_regions_img.convertTo(motion_img_send, CV_8UC3, 1.0, 0);
    motion_img_msg.image = motion_img_send;
    motion_img_msg.header = cur_camera_header_;
    motion_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    motion_img_pub_.publish(motion_img_msg.toImageMsg());

    cv::Mat arm_regions_img;
    color_frame.copyTo(arm_regions_img, arm_cut);
    cv_bridge::CvImage arm_img_msg;
    cv::Mat arm_img_send(arm_regions_img.size(), CV_8UC3);
    arm_regions_img.convertTo(arm_img_send, CV_8UC3, 1.0, 0);
    arm_img_msg.image = arm_img_send;
    arm_img_msg.header = cur_camera_header_;
    arm_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    arm_img_pub_.publish(arm_img_msg.toImageMsg());
    // cv::Mat not_arm_move = cut - arm_cut;
    // cv::Mat not_arm_move_color;
    // color_frame.copyTo(not_arm_move_color, not_arm_move);

    cv::Mat last_motion_mask;
    cv::Mat last_arm_mask;
    cv::Mat last_color_frame;
    cv::Mat last_depth_frame;
    cv::Mat last_flow_u;
    cv::Mat last_flow_v;

    cut.copyTo(last_motion_mask);
    arm_cut.copyTo(last_arm_mask);
    color_frame.copyTo(last_color_frame);
    depth_frame.copyTo(last_depth_frame);
    flow_outs[0].copyTo(last_flow_u);
    flow_outs[1].copyTo(last_flow_v);

    motion_mask_hist_.push_back(last_motion_mask);
    arm_mask_hist_.push_back(last_arm_mask);
    color_frame_hist_.push_back(last_color_frame);
    depth_frame_hist_.push_back(last_depth_frame);
    flow_u_hist_.push_back(last_flow_u);
    flow_v_hist_.push_back(last_flow_v);

    if (motion_mask_hist_.size() > image_hist_size_)
    {
      motion_mask_hist_.pop_front();
      arm_mask_hist_.pop_front();
      color_frame_hist_.pop_front();
      depth_frame_hist_.pop_front();
      flow_u_hist_.pop_front();
      flow_v_hist_.pop_front();
    }

#ifdef AUTO_FLOW_CLUSTER
    os_.getPushVector(last_motion_mask, last_arm_mask, last_color_frame,
                      last_depth_frame, last_flow_u, last_flow_v);
#endif // AUTO_FLOW_CLUSTER

#ifdef WRITE_INPUT_TO_DISK
    std::stringstream input_out_name;
    input_out_name << base_output_path_ << "input" << tracker_count_ << ".tiff";
    cv::imwrite(input_out_name.str(), color_frame);
#endif // WRITE_INPUT_TO_DISK
#ifdef WRITE_FLOWS_TO_DISK
    cv::Mat flow_thresh_disp_img(color_frame.size(), CV_8UC3);
    flow_thresh_disp_img = color_frame.clone();
    for (int r = 0; r < flow_thresh_disp_img.rows; ++r)
    {
      for (int c = 0; c < flow_thresh_disp_img.cols; ++c)
      {
        float u = flow_outs[0].at<float>(r,c);
        float v = flow_outs[1].at<float>(r,c);
        if (std::sqrt(u*u+v*v) > mgc_.magnitude_thresh_)
        {
          cv::line(flow_thresh_disp_img, cv::Point(c,r),
                   cv::Point(c-u, r-v), cv::Scalar(0,255,0));
        }
      }
    }

    std::stringstream flow_out_name;
    flow_out_name << base_output_path_ << "flow" << tracker_count_ << ".tiff";
    cv::imwrite(flow_out_name.str(), flow_thresh_disp_img);
#endif // WRITE_FLOWS_TO_DISK
#ifdef WRITE_CUTS_TO_DISK
    std::stringstream cut_out_name;
    cut_out_name << base_output_path_ << "cut" << tracker_count_ << ".tiff";
    cv::imwrite(cut_out_name.str(), moving_regions_img);
#endif // WRITE_CUTS_TO_DISK
#ifdef WRITE_ARM_CUT_TO_DISK
    std::stringstream arm_cut_out_name;
    arm_cut_out_name << base_output_path_ << "arm_cut" << tracker_count_ << ".tiff";
    cv::imwrite(arm_cut_out_name.str(), arm_regions_img);
    // std::stringstream not_arm_move_out_name;
    // not_arm_move_out_name << base_output_path_ << "not_arm_move" << tracker_count_
    //                      << ".tiff";
    // cv::imwrite(not_arm_move_out_name.str(), not_arm_move_color);
#endif // WRITE_ARM_CUT_TO_DISK
#ifdef DISPLAY_INPUT_COLOR
    std::vector<cv::Mat> hsv;
    cv::split(color_frame_f, hsv);
    cv::imshow("hue", hsv[0]);
    cv::imshow("saturation", hsv[1]);
    // cv::imshow("intensity", hsv[2]);
    // cv::imshow("input_color", color_frame);
#endif // DISPLAY_INPUT_COLOR
#ifdef DISPLAY_INPUT_DEPTH
    double depth_max = 1.0;
    cv::minMaxLoc(depth_frame, NULL, &depth_max);
    cv::Mat depth_display = depth_frame.clone();
    depth_display /= depth_max;
    cv::imshow("input_depth", depth_display);
#endif // DISPLAY_INPUT_DEPTH
#ifdef DISPLAY_OPTICAL_FLOW
    displayOpticalFlow(color_frame, flow_outs[0], flow_outs[1],
                       mgc_.magnitude_thresh_);
#endif // DISPLAY_OPTICAL_FLOW
#ifdef DISPLAY_ARM_CIRCLES
    cv::Mat arms_img(color_frame.size(), CV_8UC3);
    arms_img = color_frame.clone();
    for (unsigned int i = 0; i < hands_and_arms.size(); ++i)
    {
      for (unsigned int j = 0; j < hands_and_arms[i].size(); ++j)
      {
        cv::Scalar color;
        if (i%2 == 0)
        {
          color = cv::Scalar(0,0,255);
        }
        else
        {
          color = cv::Scalar(0,255,0);
        }
        cv::circle(arms_img, hands_and_arms[i][j], 2, color);
      }
    }
    cv::imshow("arms", arms_img);
#endif
#ifdef DISPLAY_WORKSPACE_MASK
    cv::imshow("workspace_mask", cur_workspace_mask_);
#endif // DISPLAY_WORKSPACE_MASK
#ifdef DISPLAY_GRAPHCUT
    cv::imshow("moving_regions", moving_regions_img);
    cv::imshow("arm_cut", arm_regions_img);
    // cv::imshow("not_arm_move", not_arm_move_color);
#endif // DISPLAY_GRAPHCUT

#if defined DISPLAY_INPUT_COLOR || defined DISPLAY_INPUT_DEPTH || defined DISPLAY_OPTICAL_FLOW || defined DISPLAY_GRAPHCUT || defined DISPLAY_WORKSPACE_MASK || defined DISPLAY_OPT_FLOW_INTERNALS || defined DISPLAY_GRAPHCUT || defined VISUALIZE_GRAPH_WEIGHTS || defined VISUALIZE_ARM_GRAPH_WEIGHTS || defined DISPLAY_ARM_CIRCLES || defined DISPLAY_FLOW_FIELD_CLUSTERING
    cv::waitKey(display_wait_ms_);
#endif // Any display defined
    ++tracker_count_;
    return cut;
  }

  //
  // Arm detection methods
  //
  cv::Point projectPointIntoImage(PointStamped cur_point,
                                  std::string target_frame)
  {
    cv::Point img_loc;
    try
    {
      // Transform point into the camera frame
      PointStamped image_frame_loc_m;
      tf_.transformPoint(target_frame, cur_point, image_frame_loc_m);
      // Project point onto the image
      img_loc.x = static_cast<int>((cam_info_.K[0]*image_frame_loc_m.point.x +
                                    cam_info_.K[2]*image_frame_loc_m.point.z) /
                                   image_frame_loc_m.point.z);
      img_loc.y = static_cast<int>((cam_info_.K[4]*image_frame_loc_m.point.y +
                                    cam_info_.K[5]*image_frame_loc_m.point.z) /
                                   image_frame_loc_m.point.z);

      // Downsample poses if the image is downsampled
      for (int i = 0; i < num_downsamples_; ++i)
      {
        img_loc.x /= 2;
        img_loc.y /= 2;
      }
    }
    catch (tf::TransformException e)
    {
      ROS_ERROR_STREAM(e.what());
    }
    return img_loc;
  }

  cv::Point projectPointIntoImage(PoseStamped cur_pose,
                                  std::string target_frame)
  {
    PointStamped cur_point;
    cur_point.header = cur_pose.header;
    cur_point.point = cur_pose.pose.position;
    return projectPointIntoImage(cur_point, target_frame);
  }

  bool getLineValues(cv::Point p1, cv::Point p2, std::vector<cv::Point>& line,
                     cv::Size frame_size,
                     int &min_x, int &max_x, int &min_y, int &max_y)
  {
    int num_points_added = 0;
    bool steep = (abs(p1.y - p2.y) > abs(p1.x - p2.x));
    if (steep)
    {
      // Swap x and y
      cv::Point tmp(p1.y, p1.x);
      p1.x = tmp.x;
      p1.y = tmp.y;
      tmp.y = p2.x;
      tmp.x = p2.y;
      p2.x = tmp.x;
      p2.y = tmp.y;
    }
    if (p1.x > p2.x)
    {
      // Swap p1 and p2
      cv::Point tmp(p1.x, p1.y);
      p1.x = p2.x;
      p1.y = p2.y;
      p2.x = tmp.x;
      p2.y = tmp.y;
    }
    int dx = p2.x - p1.x;
    int dy = abs(p2.y - p1.y);
    int error = dx / 2;
    int ystep = 0;
    if (p1.y < p2.y)
    {
      ystep = 1;
    }
    else if (p1.y > p2.y)
    {
      ystep = -1;
    }
    for(int x = p1.x, y = p1.y; x <= p2.x; ++x)
    {
      if (steep)
      {
        cv::Point p_new(y,x);
        // Test that p_new is in the image
        if (x < 0 || y < 0 || x >= frame_size.height || y >= frame_size.width ||
            (x == 0 && y == 0))
        {
        }
        else
        {
          if (p_new.x < min_x)
            min_x = p_new.x;
          if (p_new.x > max_x)
            max_x = p_new.x;
          if (p_new.y < min_y)
            min_y = p_new.y;
          if (p_new.y > max_y)
            max_y = p_new.y;
          line.push_back(p_new);
          ++num_points_added;
        }
      }
      else
      {
        cv::Point p_new(x,y);
        // Test that p_new is in the image
        if (x < 0 || y < 0 || x >= frame_size.width || y >= frame_size.height ||
            (x == 0 && y == 0))
        {
        }
        else
        {
          if (p_new.x < min_x)
            min_x = p_new.x;
          if (p_new.x > max_x)
            max_x = p_new.x;
          if (p_new.y < min_y)
            min_y = p_new.y;
          if (p_new.y > max_y)
            max_y = p_new.y;
          line.push_back(p_new);
          ++num_points_added;
        }
      }
      error -= dy;
      if (error < 0)
      {
        y += ystep;
        error += dx;
      }
    }
    return (num_points_added > 0);
  }

  ArmModel projectArmPoses(std_msgs::Header img_header, cv::Size frame_size,
                           int &min_x, int &max_x, int &min_y, int &max_y)
  {
    // Project all arm joints into image
    ArmModel arm_locs;

    // Left hand
    cv::Point ll0 = projectJointOriginIntoImage(img_header,
                                                "l_gripper_l_finger_tip_link");
    cv::Point ll1 = projectJointOriginIntoImage(img_header,
                                                "l_gripper_l_finger_link");
    cv::Point lr0 = projectJointOriginIntoImage(img_header,
                                                "l_gripper_r_finger_tip_link");
    cv::Point lr1 = projectJointOriginIntoImage(img_header,
                                                "l_gripper_r_finger_link");
    cv::Point lp = projectJointOriginIntoImage(img_header,
                                               "l_gripper_palm_link");
    // TODO: Add more arm locations
    // Left arm
    cv::Point l1 = projectJointOriginIntoImage(img_header, "l_wrist_flex_link");
    cv::Point l2 = projectJointOriginIntoImage(img_header, "l_forearm_link");
    cv::Point l3 = projectJointOriginIntoImage(img_header, "l_upper_arm_link");
    arm_locs.l_chain.push_back(lp);
    arm_locs.l_chain.push_back(ll1);
    arm_locs.l_chain.push_back(ll0);
    arm_locs.l_chain.push_back(lr0);
    arm_locs.l_chain.push_back(lr1);
    arm_locs.l_chain.push_back(lp);
    arm_locs.l_chain.push_back(l1);
    arm_locs.l_chain.push_back(l2);
    arm_locs.l_chain.push_back(l3);

    // Right hand
    cv::Point rl0 = projectJointOriginIntoImage(img_header,
                                                "r_gripper_l_finger_tip_link");
    cv::Point rl1 = projectJointOriginIntoImage(img_header,
                                                "r_gripper_l_finger_link");
    cv::Point rr0 = projectJointOriginIntoImage(img_header,
                                                "r_gripper_r_finger_tip_link");
    cv::Point rr1 = projectJointOriginIntoImage(img_header,
                                                "r_gripper_r_finger_link");
    cv::Point rp = projectJointOriginIntoImage(img_header,
                                               "r_gripper_palm_link");

    // TODO: Add more arm locations
    // Right arm
    cv::Point r1 = projectJointOriginIntoImage(img_header, "r_wrist_flex_link");
    cv::Point r2 = projectJointOriginIntoImage(img_header, "r_forearm_link");
    cv::Point r3 = projectJointOriginIntoImage(img_header, "r_upper_arm_link");
    arm_locs.r_chain.push_back(rp);
    arm_locs.r_chain.push_back(rl1);
    arm_locs.r_chain.push_back(rl0);
    arm_locs.r_chain.push_back(rr0);
    arm_locs.r_chain.push_back(rr1);
    arm_locs.r_chain.push_back(rp);
    arm_locs.r_chain.push_back(r1);
    arm_locs.r_chain.push_back(r2);
    arm_locs.r_chain.push_back(r3);

    // Keep track of min and max values
    min_x = 10000;
    max_x = 0;
    min_y = 10000;
    max_y = 0;

    // Add left hand
    arm_locs.l_hand_on = getLineValues(ll0, ll1, arm_locs.hands, frame_size,
                                       min_x, max_x, min_y, max_y);
    arm_locs.l_hand_on = (getLineValues(ll1, lp, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                          arm_locs.l_hand_on);
    arm_locs.l_hand_on = (getLineValues(lr0, lr1, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                          arm_locs.l_hand_on);
    arm_locs.l_hand_on = (getLineValues(lr1, lp, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                          arm_locs.l_hand_on);
    // Add left arm
    arm_locs.l_arm_on = getLineValues(lp, l1, arm_locs.arms, frame_size,
                                      min_x, max_x, min_y, max_y);
    arm_locs.l_arm_on = (getLineValues(l1, l2, arm_locs.arms, frame_size,
                                      min_x, max_x, min_y, max_y) ||
                         arm_locs.l_arm_on);
    arm_locs.l_arm_on = (getLineValues(l2, l3, arm_locs.arms, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.l_arm_on);
    // Add right hand
    arm_locs.r_hand_on = (getLineValues(rl0, rl1, arm_locs.hands, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.r_hand_on);
    arm_locs.r_hand_on = (getLineValues(rl1, rp, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                         arm_locs.r_hand_on);
    arm_locs.r_hand_on = (getLineValues(rr0, rr1, arm_locs.hands, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.r_hand_on);
    arm_locs.r_hand_on = (getLineValues(rr1, rp, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                         arm_locs.r_hand_on);
    // Add right arm
    arm_locs.r_arm_on = getLineValues(rp, r1, arm_locs.arms, frame_size,
                                      min_x, max_x, min_y, max_y);
    arm_locs.r_arm_on = (getLineValues(r1, r2, arm_locs.arms, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.r_arm_on);
    arm_locs.r_arm_on = (getLineValues(r2, r3, arm_locs.arms, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.r_arm_on);
    return arm_locs;
  }

  cv::Point projectJointOriginIntoImage(std_msgs::Header img_header,
                                        std::string joint_name)
  {
    PointStamped joint_origin;
    joint_origin.header.frame_id = joint_name;
    joint_origin.header.stamp = img_header.stamp;
    joint_origin.point.x = 0.0;
    joint_origin.point.y = 0.0;
    joint_origin.point.z = 0.0;
    // TODO: Use more information in setting correct locations
    if (joint_name == "r_gripper_tool_frame" ||
        joint_name == "l_gripper_tool_frame" ||
        joint_name == "l_gripper_l_finger_tip_link" ||
        joint_name == "l_gripper_l_finger_link" ||
        joint_name == "l_gripper_r_finger_tip_link" ||
        joint_name == "l_gripper_r_finger_link" ||
        joint_name == "r_gripper_l_finger_tip_link" ||
        joint_name == "r_gripper_l_finger_link" ||
        joint_name == "r_gripper_r_finger_tip_link" ||
        joint_name == "r_gripper_r_finger_link" ||
        joint_name == "r_gripper_tool_frame")
    {
      joint_origin.point.z = 0.01;
      if (joint_name == "r_gripper_l_finger_link" ||
          joint_name == "l_gripper_l_finger_link")
      {
        joint_origin.point.y = 0.02;
      }
      else if (joint_name == "r_gripper_r_finger_link" ||
               joint_name == "l_gripper_r_finger_link")
      {
        joint_origin.point.y = -0.02;
      }
    }
    // TODO: Check this better
    if (joint_name == "r_upper_arm_link")
    {
      joint_origin.point.y = -0.05;
    }
    // TODO: Check this better
    if (joint_name == "l_upper_arm_link")
    {
      joint_origin.point.y = 0.05;
    }
    if (joint_name == "r_gripper_palm_link")
    {
      joint_origin.point.x = 0.05;
    }
    if (joint_name == "l_gripper_palm_link")
    {
      joint_origin.point.x = 0.05;
    }
    return projectPointIntoImage(joint_origin, img_header.frame_id);
  }

  void drawTablePlaneOnImage(XYZPointCloud& plane, PoseStamped cent)
  {
    cv::Mat plane_display = cur_color_frame_.clone();
    std::vector<cv::Point> table_points;
    for (unsigned int i = 0; i < plane.points.size(); ++i)
    {
      PointStamped h;
      h.header = plane.header;
      h.point.x = plane.points[i].x;
      h.point.y = plane.points[i].y;
      h.point.z = plane.points[i].z;
      cv::Point p = projectPointIntoImage(h, cur_camera_header_.frame_id);
      table_points.push_back(p);
#ifdef DISPLAY_PLANE_ESTIMATE
      cv::circle(plane_display, p, 2, cv::Scalar(0,255,0));
#endif // DISPLAY_PLANE_ESTIMATE
    }

#ifdef USE_TABLE_COLOR_ESTIMATE
    ROS_INFO_STREAM("Calculating table color stats.");
    mgc_.setTableColorStats(cur_color_frame_, table_points);
#endif // USE_TABLE_COLOR_ESTIMATE

#ifdef DISPLAY_PLANE_ESTIMATE
    cv::Point cent_img = projectPointIntoImage(cent,
                                               cur_camera_header_.frame_id);
    cv::circle(plane_display, cent_img, 5, cv::Scalar(0,0,255));
    cv::imshow("table_image", plane_display);
#endif // DISPLAY_PLANE_ESTIMATE
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

  cv::Mat getTableHeightDistances()
  {
    cv::Mat table_distances(cur_depth_frame_.size(), CV_32FC1, cv::Scalar(0.0));
    const float table_height = table_centroid_.pose.position.z;
    for (int r = 0; r < table_distances.rows; ++r)
    {
      for (int c = 0; c < table_distances.cols; ++c)
      {
        pcl::PointXYZ cur_pt = cur_point_cloud_.at(2*c, 2*r);
        if (isnan(cur_pt.x) || isnan(cur_pt.y) || isnan(cur_pt.z))
        {
          // 3 meters is sufficiently far away
          table_distances.at<float>(r,c) = 3.0;
        }
        else
        {
          table_distances.at<float>(r,c) = cur_pt.z - table_height;
        }
      }
    }
#ifdef DISPLAY_TABLE_DISTANCES
    cv::imshow("table_distances_raw", table_distances);
#endif // DISPLAY_TABLE_DISTANCES
    return table_distances;
  }

  PoseStamped getTablePlane(XYZPointCloud& cloud)
  {
    // Filter Cloud to not look for table planes on the ground
    XYZPointCloud cloud_z_filtered;
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setInputCloud(
        boost::make_shared<XYZPointCloud>(cloud));
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
    plane_seg.setDistanceThreshold (table_ransac_thresh_);
    plane_seg.setInputCloud (
        boost::make_shared<XYZPointCloud>(cloud_z_filtered));
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
    XYZPointCloud plane_cloud;
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
    drawTablePlaneOnImage(plane_cloud, p);
    return p;
  }

  PoseStamped findRandomPushPose(XYZPointCloud& input_cloud)
  {
    XYZPointCloud cloud_filtered, cloud_z_filtered;
    ROS_INFO_STREAM("input_cloud.size() " << input_cloud.size());
    ROS_INFO_STREAM("input_cloud.header.frame_id() "
                    << input_cloud.header.frame_id);

    // Filter Cloud to be just table top height
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setInputCloud(boost::make_shared<XYZPointCloud>(input_cloud));
    z_pass.setFilterFieldName ("z");
    z_pass.setFilterLimits(min_table_z_, max_table_z_);
    z_pass.filter(cloud_z_filtered);
    ROS_INFO_STREAM("Filtered z");
    ROS_INFO_STREAM("cloud_z_filtered.size() " << cloud_z_filtered.size());

    // Filter to be just in the range in front of the robot
    pcl::PassThrough<pcl::PointXYZ> x_pass;
    x_pass.setInputCloud(
        boost::make_shared<XYZPointCloud >(cloud_z_filtered));
    x_pass.setFilterFieldName ("x");
    x_pass.setFilterLimits(min_workspace_x_, max_workspace_x_);
    x_pass.filter(cloud_filtered);
    ROS_INFO_STREAM("Filtered x");
    ROS_INFO_STREAM("cloud_filtered.size() " << cloud_filtered.size());

    // Segment the tabletop from the points using RANSAC plane fitting
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices plane_inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold (table_ransac_thresh_);
    plane_seg.setInputCloud(boost::make_shared<XYZPointCloud >(cloud_filtered));
    plane_seg.segment(plane_inliers, coefficients);

    // Extract the plane members into their own point cloud and save them
    // into a PCD file
    XYZPointCloud plane_cloud;
    pcl::copyPointCloud(cloud_filtered, plane_inliers, plane_cloud);
    ROS_INFO_STREAM("plane_cloud.size() " << plane_cloud.size());
    Eigen::Vector4f plane_xyz_centroid;
    pcl::compute3DCentroid(plane_cloud, plane_xyz_centroid);
    ROS_INFO_STREAM("Plane centroid is: (" << plane_xyz_centroid[0]
                    << ", " << plane_xyz_centroid[1]
                    << ", " << plane_xyz_centroid[2] << ")");

    // Extract the outliers from the point clouds
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    XYZPointCloud objects_cloud;
    pcl::PointIndices plane_outliers;
    extract.setInputCloud (
        boost::make_shared<XYZPointCloud > (cloud_filtered));
    extract.setIndices (boost::make_shared<pcl::PointIndices> (plane_inliers));
    extract.setNegative (true);
    extract.filter(objects_cloud);

    // TODO: This would be better if we first filter out points below the plane,
    // so that only objects on top of the table are clustered

    // Cluster the objects based on euclidean distance
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_cluster;
    KdTreePtr clusters_tree =
        boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
    pcl_cluster.setClusterTolerance(0.025);
    pcl_cluster.setMinClusterSize(100);
    pcl_cluster.setSearchMethod(clusters_tree);
    pcl_cluster.setInputCloud(
        boost::make_shared<XYZPointCloud >(objects_cloud));
    pcl_cluster.extract (clusters);
    ROS_INFO_STREAM("Number of clusters found matching the given constraints: "
                    << clusters.size());

    std::vector<Eigen::Vector4f> cluster_centroids;
    for (unsigned int i = 0; i < clusters.size(); ++i)
    {
      // Choose a random cluster to push
      Eigen::Vector4f obj_xyz_centroid;
      XYZPointCloud obj_cloud;
      pcl::copyPointCloud(objects_cloud, clusters[i], obj_cloud);
      pcl::compute3DCentroid(obj_cloud, obj_xyz_centroid);

      if (obj_xyz_centroid[0] > min_pushing_x_ &&
          obj_xyz_centroid[0] < max_pushing_x_ &&
          obj_xyz_centroid[1] > min_pushing_y_ &&
          obj_xyz_centroid[1] < max_pushing_y_)
      {
        cluster_centroids.push_back(obj_xyz_centroid);
      }
    }
    geometry_msgs::PoseStamped p;
    if (cluster_centroids.size() < 1)
    {
      ROS_ERROR_STREAM("No object clusters found! Returning empty push_pose");
      return p;
    }
    ROS_INFO_STREAM("Found " << cluster_centroids.size() << " proto objects");
    int rand_idx = rand() % cluster_centroids.size();
    Eigen::Vector4f obj_xyz_centroid = cluster_centroids[rand_idx];
    ROS_INFO_STREAM("Chosen centroid is at: (" << obj_xyz_centroid[0] << ", "
                    << obj_xyz_centroid[1] << ", " << obj_xyz_centroid[2] << ")");
    p.pose.position.x = obj_xyz_centroid[0];
    p.pose.position.y = obj_xyz_centroid[1];
    // Set z to be the table height
    p.pose.position.z = plane_xyz_centroid[2];

    // TODO: Find orientation from the centroid
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 0;

    p.header.frame_id = "torso_lift_link";
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
  ros::NodeHandle n_private_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Synchronizer<MySyncPolicy> sync_;
  image_transport::ImageTransport it_;
  sensor_msgs::CameraInfo cam_info_;
  image_transport::Publisher motion_img_pub_;
  image_transport::Publisher motion_mask_pub_;
  image_transport::Publisher arm_img_pub_;
  image_transport::Publisher arm_mask_pub_;
  actionlib::SimpleActionServer<tabletop_pushing::SegTrackAction> track_server_;
  sensor_msgs::CvBridge bridge_;
  tf::TransformListener tf_;
  ros::ServiceServer push_pose_server_;
  ros::ServiceServer table_location_server_;
  cv::Mat cur_color_frame_;
  cv::Mat cur_depth_frame_;
  cv::Mat cur_workspace_mask_;
  cv::Mat prev_color_frame_;
  cv::Mat prev_depth_frame_;
  cv::Mat prev_workspace_mask_;
  cv::Mat prev_seg_mask_;
  std::deque<cv::Mat> motion_mask_hist_;
  std::deque<cv::Mat> arm_mask_hist_;
  std::deque<cv::Mat> color_frame_hist_;
  std::deque<cv::Mat> depth_frame_hist_;
  std::deque<cv::Mat> flow_u_hist_;
  std::deque<cv::Mat> flow_v_hist_;
  std_msgs::Header cur_camera_header_;
  std_msgs::Header prev_camera_header_;
  XYZPointCloud cur_point_cloud_;
  LKFlowReliable lkflow_;
  MotionGraphcut mgc_;
  ObjectSingulation os_;
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
  double min_pushing_x_;
  double max_pushing_x_;
  double min_pushing_y_;
  double max_pushing_y_;
  double min_table_z_;
  double max_table_z_;
  double below_table_z_;
  int num_downsamples_;
  std::string workspace_frame_;
  PoseStamped table_centroid_;
  bool tracking_;
  bool tracker_initialized_;
  std::string cam_info_topic_;
  int tracker_count_;
  std::string base_output_path_;
  double table_ransac_thresh_;
  int image_hist_size_;
};

int main(int argc, char ** argv)
{
  srand(time(NULL));
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  TabletopPushingPerceptionNode perception_node(n);
  perception_node.spin();

  return 0;
}
