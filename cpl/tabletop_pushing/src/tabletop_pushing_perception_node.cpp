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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

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
#include <pcl/features/feature.h>
#include <pcl/common/eigen.h>
#include "pcl/filters/voxel_grid.h"
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
#include <math.h>

// Debugging IFDEFS
// #define DISPLAY_INPUT_COLOR 1
// #define DISPLAY_INPUT_DEPTH 1
// #define DISPLAY_WORKSPACE_MASK 1
// #define DISPLAY_OPTICAL_FLOW 1
// #define DISPLAY_UV 1
// #define DISPLAY_OPT_FLOW_INTERNALS 1
// #define DISPLAY_GRAPHCUT 1
// #define VISUALIZE_GRAPH_WEIGHTS 1
// #define VISUALIZE_GRAPH_EDGE_WEIGHTS 1
// #define VISUALIZE_ARM_GRAPH_WEIGHTS 1
// #define VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS 1
// #define DISPLAY_ARM_CIRCLES 1

// #define WRITE_INPUT_TO_DISK 1
// #define WRITE_CUTS_TO_DISK 1
// #define WRITE_FLOWS_TO_DISK 1
// #define WRITE_ARM_CUT_TO_DISK 1

// Functional IFDEFS
// #define REMOVE_SMALL_BLOBS 1
// #define FAKE_SEGMENT
#define MEDIAN_FILTER_FLOW 1

using tabletop_pushing::PushPose;
using tabletop_pushing::LocateTable;
using geometry_msgs::PoseStamped;
using geometry_msgs::PointStamped;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2> MySyncPolicy;
typedef Graph<float, float, float> GraphType;

inline const float max(const float a, const double b)
{
  return max(static_cast<double>(a), b);
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
        // cv::circle(flow_thresh_disp_img, cv::Point(c,r), 2, cv::Scalar(0,255,0));
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
    if (i == 1)
      return arms;
    // else
    //   throw e;
  }

  /**
   * Return the distance to the closest point on the arm from an image point p
   *
   * @param p The point in the image
   *
   * @return The distance to the closest point on any in-image arm
   */
  float distanceToArm(cv::Point2f p)
  {
    float l_dist = distanceToArm(p, l_chain);
    float r_dist = distanceToArm(p, r_chain);
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

  float distanceToArm(cv::Point2f p, std::vector<cv::Point>& chain)
  {
    if (chain.size() == 0)
    {
      return -1.0f;
    }
    float min_dist = 640.0f*480.0f;
    for (unsigned int i = 1; i < chain.size(); ++i)
    {
      float d_i = pointLineDistance(p, chain[i-1] , chain[i]);
      if (d_i < min_dist)
        min_dist = d_i;
    }
    return min_dist;
  }

  float pointLineDistance(cv::Point2f p, cv::Point2f l0, cv::Point2f l1)
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
      return hypot(p.x - q.x, p.y - q.y);
    }
    cv::Point2f* x0;
    cv::Point2f* x1;

    if (l0.x < l1.x)
    {
      x0 = &l0;
      x1 = &l1;
    }
    else
    {
      x0 = &l1;
      x1 = &l0;
    }
    cv::Point2f v = *x1 - *x0;
    cv::Point2f w = p - *x0;

    float c0 = w.x*v.x+w.y*v.y;
    float c1 = v.x*v.x+v.y*v.y;
    float b = c0/c1;

    cv::Point2f q = *x0 + b*v;
    float d = hypot(p.x - q.x, p.y - q.y);
    if (c0 <= 0 || q.x < x0->x)
    {
      d = hypot(p.x - x0->x, p.y - x0->y);
    }
    if (c1 <= 0 || q.x > x1->x)
    {
      d = hypot(p.x - x1->x, p.y - x1->y);
    }
    return d;
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

class LKFlowReliable
{
 public:
  LKFlowReliable(int win_size = 5, int num_levels = 4) :
      win_size_(win_size), max_level_(num_levels-1)
  {
    // Create derivative kernels for flow calculation
    cv::getDerivKernels(dy_kernel_, dx_kernel_, 1, 0, CV_SCHARR, true, CV_32F);
    // TODO: Make directly
    // [+3 +10 +3
    //   0   0  0
    //  -3 -10 -3]
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
    total_outs.push_back(flow_outs[2]);
    return total_outs;
  }

  // TODO: Clean up this method to be more like optic.m
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

#ifdef DISPLAY_OPT_FLOW_INTERNALS
    cv::imshow("cur_bw", cur_bw);
    cv::imshow("prev_bw", prev_bw);
    cv::imshow("It", It);
    cv::imshow("Ix", Ix);
    cv::imshow("Iy", Iy);
#endif // DISPLAY_OPT_FLOW_INTERNALS

    int win_radius = win_size_/2;
    cv::Mat flow_u(cur_bw.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat flow_v(cur_bw.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat t_scores(cur_bw.size(), CV_32FC1, cv::Scalar(0.0));
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

        float det = sIxx*sIyy - sIxy*sIxy;
        cv::Vec2f uv;
        if (det == 0.0)
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
        t_scores.at<float>(r,c) = (sIxx+sIyy)*(sIxx+sIyy)/(det);
      }
    }
    std::vector<cv::Mat> outs;
    outs.push_back(flow_u);
    outs.push_back(flow_v);
    outs.push_back(t_scores);
    return outs;
  }

  cv::Mat reduce(cv::Mat& input /*, cv::Mat kernel*/)
  {
    cv::Mat output;// = input.clone();
    cv::pyrDown(input, output);
    return output;
  }

  cv::Mat expand(cv::Mat& input /*, cv::Mat kernel*/)
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

  void integralImage(cv::Mat& input)
  {
    // Compute integral image first row of values
    double* row0 = input.ptr<double>(0);
    for (int c = 1; c < input.cols; ++c)
    {
      row0[c] += row0[c-1];
    }

    // Compute integral image first column of values
    for (int r = 1; r < input.rows; ++r)
    {
      input.at<double>(r,0) += input.at<double>(r-1,0);
    }

    // Compute integral image values
    double* prev_row = row0;
    double* cur_row;
    for (int r = 1; r < input.rows; ++r)
    {
      cur_row = input.ptr<double>(r);
      for (int c = 1; c < input.cols; ++c)
      {
        cur_row[c] += cur_row[c-1] + prev_row[c] - prev_row[c-1];
      }
      prev_row = cur_row;
    }
  }

  inline double filterIntegral(cv::Mat& integral, int x, int y, int radius)
  {
    int y_min = y - radius - 1;
    int x_min = x - radius - 1;
    int y_max = y + radius;
    int x_max = x + radius;

    double filter_response = integral.at<double>(y_max, x_max);
    if (x_min >= 0)
    {
      filter_response -= integral.at<double>(y_max, x_min);
    }
    if (y_min >= 0)
    {
      filter_response -= integral.at<double>(y_min, x_max);
    }
    if (x_min >= 0 && y_min >= 0)
    {
      filter_response += integral.at<double>(y_min, x_min);
    }
    return filter_response;
  }

  std::vector<cv::Mat> baseLKII(cv::Mat& cur_bw, cv::Mat& prev_bw)
  {

    // Blur the input images and get the difference
    cv::Mat cur_blur(cur_bw.size(), cur_bw.type());
    cv::Mat prev_blur(prev_bw.size(), prev_bw.type());
    cv::filter2D(cur_bw, cur_blur, CV_32F, g_kernel_);
    cv::filter2D(prev_bw, prev_blur, CV_32F, g_kernel_);
    cv::Mat It32 = cur_blur - prev_blur;

    // Get image derivatives
    cv::Mat Ix32(cur_bw.size(), CV_32FC1);
    cv::Mat Iy32(cur_bw.size(), CV_32FC1);
    cv::filter2D(cur_bw, Ix32, CV_32F, dx_kernel_);
    cv::filter2D(cur_bw, Iy32, CV_32F, dy_kernel_);

    // Convert images to 64 bit to preserve precision in integration
    cv::Mat Ix(cur_bw.size(), CV_64FC1);
    cv::Mat Iy(cur_bw.size(), CV_64FC1);
    cv::Mat It(It32.size(), CV_64FC1);
    Ix32.convertTo(Ix, CV_64FC1);
    Iy32.convertTo(Iy, CV_64FC1);
    It32.convertTo(It, CV_64FC1);

    // Create the integral images
    cv::Mat IIxx(cur_bw.size(), CV_64FC1);
    cv::Mat IIyy(cur_bw.size(), CV_64FC1);
    cv::Mat IIxy(cur_bw.size(), CV_64FC1);
    cv::Mat IIxt(cur_bw.size(), CV_64FC1);
    cv::Mat IIyt(cur_bw.size(), CV_64FC1);
    cv::multiply(Ix, Ix, IIxx);
    cv::multiply(Iy, Iy, IIyy);
    cv::multiply(Ix, Iy, IIxy);
    cv::multiply(Ix, It, IIxt);
    cv::multiply(Iy, It, IIyt);
    integralImage(IIxx);
    integralImage(IIyy);
    integralImage(IIxy);
    integralImage(IIxt);
    integralImage(IIyt);

#ifdef DISPLAY_OPT_FLOW_INTERNALS
    cv::imshow("IIxx", IIxx);
    cv::imshow("IIxy", IIxy);
    cv::imshow("IIyy", IIyy);
    cv::imshow("IIxt", IIxt);
    cv::imshow("IIyt", IIyt);
    cv::imshow("cur_bw", cur_blur);
    cv::imshow("prev_bw", prev_blur);
    cv::imshow("It", It);
    cv::imshow("Ix", Ix);
    cv::imshow("Iy", Iy);
#endif // DISPLAY_OPT_FLOW_INTERNALS

    // Compute the LK flow densely using the integral images
    int win_radius = win_size_/2;
    cv::Mat flow(cur_bw.size(), CV_64FC2, cv::Scalar(0.0,0.0));
    cv::Mat t_scores(cur_bw.size(), CV_32FC1, cv::Scalar(0.0,0.0));
    for (int r = win_radius; r < Ix.rows-win_radius; ++r)
    {
      for (int c = win_radius; c < Ix.cols-win_radius; ++c)
      {
        const double sIxx = filterIntegral(IIxx, c, r, win_radius);
        const double sIyy = filterIntegral(IIyy, c, r, win_radius);
        const double sIxy = filterIntegral(IIxy, c, r, win_radius);
        const double sIxt = filterIntegral(IIxt, c, r, win_radius);
        const double sIyt = filterIntegral(IIyt, c, r, win_radius);
        float det = sIxx*sIyy - sIxy*sIxy;
        cv::Vec2f uv;
        if (det == 0.0)
        {
          uv[0] = 0.0;
          uv[1] = 0.0;
        }
        else
        {
          uv[0] = (-sIyy*sIxt + sIxy*sIyt)/det;
          uv[1] = (sIxy*sIxt - sIxx*sIyt)/det;
        }
        flow.at<cv::Vec2f>(r,c) = uv;
        t_scores.at<float>(r,c) = (sIxx+sIyy)*(sIxx+sIyy)/(det);
      }
    }
    std::vector<cv::Mat> outs;
    outs.push_back(flow);
    outs.push_back(t_scores);
    return outs;
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

 protected:
  int win_size_;
  int max_level_;
  cv::Mat dx_kernel_;
  cv::Mat dy_kernel_;
  cv::Mat g_kernel_;
  cv::Mat optic_g_kernel_;
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
                     cv::Mat& u, cv::Mat& v, cv::Mat eigen_scores,
                     cv::Mat& workspace_mask, ArmModel arm_locs)
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
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
    cv::Mat left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS

    // TODO: Set low probability of stuff close to table height?
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
#endif // VISUALIZE_GRAPH_WEIGHTS
          continue;
        }
        const float mag_score = max(getFlowFGScore(magnitude), min_weight_);
        const float not_mag_score = max(1.0 - mag_score, min_weight_);
        g->add_tweights(r*C+c, mag_score, not_mag_score);
#ifdef VISUALIZE_GRAPH_WEIGHTS
        fg_weights.at<float>(r,c) = mag_score;
        bg_weights.at<float>(r,c) = not_mag_score;
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
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS
    cv::Mat left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_ARM_GRAPH_WEIGHTS

    // One gaussian estimated from wrist to elbow, elbow to forearm and a
    // separate one is estimated from the gripper tip to wrist
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
//         // Check if we are hardcoding this spot to background
//         if (workspace_mask.at<uchar>(r,c) == -1)
//         {
//           g->add_tweights(r*C+c, min_weight_, workspace_background_weight_);
// #ifdef VISUALIZE_ARM_GRAPH_WEIGHTS
//           fg_weights.at<float>(r,c) = min_weight_;
//           bg_weights.at<float>(r,c) = workspace_background_weight_;
// #endif // VISUALIZE_ARM_GRAPH_WEIGHTS
//           continue;
//         }
//         else
//         {
        const float me_score = max(getArmFGScore(color_frame, r, c, arm_stats,
                                                 hand_stats, arms, roi),
                                   min_weight_);
        const float not_me_score = max(1.0 - me_score, min_weight_);
        g->add_tweights(r*C+c, /*capacities*/ me_score, not_me_score);
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
    // TODO: Ensure a single continuous region for each arm seeded by the known
    // locations
    return segs;
  }

  cv::Mat multilabelArmMovement(cv::Mat& color_frame, cv::Mat& depth_frame,
                                cv::Mat& u, cv::Mat& v, cv::Mat& workspace_mask,
                                ArmModel arms)
  {
    // NOTE: We examine only a subwindow in the image to avoid too make things
    // more efficient
    cv::Rect roi(0, 0, color_frame.cols, color_frame.rows);
    const int R = color_frame.rows;
    const int C = color_frame.cols;

    int num_nodes = R*C;
    int num_edges = ((C-1)*3+1)*(R-1)+(C-1);
    GraphType *g;
    g = new GraphType(num_nodes, num_edges);

#ifdef VISUALIZE_GRAPH_WEIGHTS
    cv::Mat fg_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat bg_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
    cv::Mat left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat up_left_weights(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS

    // One gaussian estimated from wrist to elbow, elbow to forearm and a
    // separate one is estimated from the gripper tip to wrist
    std::vector<cv::Vec3f> hand_stats = getImagePointGaussian(color_frame,
                                                              arms[0]);
    std::vector<cv::Vec3f> arm_stats = getImagePointGaussian(color_frame,
                                                             arms[1]);
    // Tie weights to fg / bg
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        g->add_node();
        // Check if we are hardcoding this spot to background
        if (workspace_mask.at<uchar>(r,c) == 0)
        {
          g->add_tweights(r*C+c, min_weight_, workspace_background_weight_);
#ifdef VISUALIZE_GRAPH_WEIGHTS
          fg_weights.at<float>(r,c) = min_weight_;
          bg_weights.at<float>(r,c) = workspace_background_weight_;
#endif // VISUALIZE_GRAPH_WEIGHTS
          continue;
        }
        else
        {
          const float magnitude = std::sqrt(u.at<float>(r,c)*u.at<float>(r,c) +
                                            v.at<float>(r,c)*v.at<float>(r,c));
          const float move_score = max(getFlowFGScore(magnitude), min_weight_);
          const float me_score = max(getArmFGScore(color_frame, r, c, arm_stats,
                                                   hand_stats, arms, roi),
                                     min_weight_);
          // TODO: Get motion score
          // TODO: Get tabletop score
          // TODO: Determine background score
#ifdef VISUALIZE_GRAPH_WEIGHTS
          fg_weights.at<float>(r,c) = me_score;
          // bg_weights.at<float>(r,c) = not_me_score;
#endif // VISUALIZE_GRAPH_WEIGHTS
        }
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
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
          left_weights.at<float>(r,c) = w_l;
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS
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
    cv::imshow("fg_weights_arm", fg_weights);
    cv::imshow("bg_weights_arm", bg_weights);
#endif // VISUALIZE_GRAPH_WEIGHTS
#ifdef VISUALIZE_GRAPH_EDGE_WEIGHTS
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
#endif // VISUALIZE_GRAPH_EDGE_WEIGHTS

    // int flow = g->maxflow(false);
    g->maxflow(false);

    // Convert output into image
    cv::Mat segs = convertFlowResultsToCvMat(g, R, C);
    delete g;
    // TODO: Ensure a single continuous region for each arm seeded by the known
    // locations
    return segs;
  }

  float getFlowFGScore(float magnitude)
  {
    return min(1.0, max( flow_gain_*exp(magnitude), 0.0));
  }

  float getArmFGScore(cv::Mat& color_frame, int r, int c,
                      std::vector<cv::Vec3f>& arm_stats,
                      std::vector<cv::Vec3f>& hand_stats, ArmModel& arms,
                      cv::Rect& roi)
  {
    const float dist_var = 20.0;
    const float dist_score = exp(-arms.distanceToArm(
        cv::Point2f(c+roi.x,r+roi.y))/dist_var);
    cv::Vec3f cur_c = color_frame.at<cv::Vec3f>(r,c);
    const float arm_h_score = 1.0-fabs(cur_c[0] - arm_stats[0][0])/(
        arm_stats[1][0] + 0.1);
    const float arm_s_score = 1.0-fabs(cur_c[1] - arm_stats[0][1])/(
        arm_stats[1][1] + 0.1);
    const float arm_score = (arm_h_score + arm_s_score)/2.0+0.5*dist_score;
    const float hand_h_score = 1.0-fabs(cur_c[0] - hand_stats[0][0])/(
        hand_stats[1][0] + 0.1);
    const float hand_s_score = 1.0-fabs(cur_c[1] - hand_stats[0][1])/(
        hand_stats[1][1] + 0.1);
    const float hand_score = (hand_h_score + hand_s_score)/2.0 + 0.5*dist_score;
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

  cv::Mat convertFlowResultsToCvMat(GraphType *g, int R, int C)
  {
    cv::Mat segs(R, C, CV_32FC1, cv::Scalar(0.0));
    for (int r = 0; r < R; ++r)
    {
      float* seg_row = segs.ptr<float>(r);
      for (int c = 0; c < C; ++c)
      {
        float label = (g->what_segment(r*C+c) == GraphType::SOURCE);
        seg_row[c] = label;
      }
    }
    return segs;
  }

  cv::Mat convertFlowResultsToCvMat(GraphType *g, int R, int C,
                                    cv::Rect roi, cv::Size out_size)
  {
    cv::Mat segs(out_size, CV_32FC1, cv::Scalar(0.0));
    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        float label = (g->what_segment(r*C+c) == GraphType::SOURCE);
        segs.at<float>(r+roi.y, c+roi.x) = label;
      }
    }
    return segs;
  }

  float getEdgeWeight(cv::Vec3f c0, float d0, cv::Vec3f c1, float d1)
  {
    cv::Vec3f c_d = c0-c1;
    float w_d = d0-d1;
    // float w_c = sqrt(c_d[0]*c_d[0]+c_d[1]*c_d[1]+c_d[2]*c_d[2] + w_d*w_d);
    // float w_c = sqrt(c_d[0]*c_d[0] + c_d[1]*c_d[1] + w_d*w_d);
    // float w_c = sqrt(c_d[0]*c_d[0] + w_d*w_d);
    float w_c = w_c_alpha_*exp(fabs(c_d[0])) + w_c_beta_*exp(fabs(c_d[1])) +
        w_c_gamma_*exp(fabs(w_d));
    return w_c;
  }

  double workspace_background_weight_;
  double min_weight_;
  double w_c_alpha_;
  double w_c_beta_;
  double w_c_gamma_;
  double magnitude_thresh_;
  double flow_gain_;
  int arm_grow_radius_;
  int arm_search_radius_;
};

class TabletopPushingPerceptionNode
{
 public:
  TabletopPushingPerceptionNode(ros::NodeHandle &n) :
      n_(n),
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
      tracker_initialized_(false), roi_(0,0,640,480), min_contour_size_(30),
      tracker_count_(0)
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
    n_private.param("autostart_tracking", tracking_, false);
    n_private.param("min_contour_size", min_contour_size_, 30);
    n_private.param("num_downsamples", num_downsamples_, 2);
    std::string cam_info_topic_def = "/kinect_head/rgb/camera_info";
    n_private.param("cam_info_topic", cam_info_topic_,
                    cam_info_topic_def);

    base_output_path_ = "/home/thermans/sandbox/cut_out/";
    // Graphcut weights
    n_private.param("mgc_workspace_bg_weight",
                    mgc_.workspace_background_weight_, 1.0);
    n_private.param("mgc_min_weight", mgc_.min_weight_, 0.01);
    n_private.param("mgc_w_c_alpha", mgc_.w_c_alpha_, 0.1);
    n_private.param("mgc_w_c_beta",  mgc_.w_c_beta_, 0.1);
    n_private.param("mgc_w_c_gamma", mgc_.w_c_gamma_, 0.1);
    n_private.param("mgc_arm_grow_radius", mgc_.arm_grow_radius_, 2);
    n_private.param("mgc_arm_search_radius", mgc_.arm_search_radius_, 50);
    // Lucas Kanade params
    n_private.param("mgc_magnitude_thresh", mgc_.magnitude_thresh_, 0.1);
    n_private.param("mgc_flow_gain", mgc_.flow_gain_, 0.3);
    int win_size = 5;
    n_private.param("lk_win_size", win_size, 5);
    lkflow_.setWinSize(win_size);
    int num_levels = 4;
    n_private.param("lk_num_levels", num_levels, 4);
    lkflow_.setNumLevels(num_levels);

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
    roi_.x = crop_min_x_;
    roi_.y = crop_min_y_;
    roi_.width = crop_max_x_ - crop_min_x_;
    roi_.height = crop_max_y_ - crop_min_y_;
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
    if (tracking_)
    {
      cv::Mat motion_mask = segmentMovingStuff(cur_color_frame_,
                                               cur_depth_frame_,
                                               prev_color_frame_,
                                               prev_depth_frame_,
                                               cur_point_cloud_);

      // TODO: Process the moving stuff to separate arm from other stuff
      prev_motion_mask_ = motion_mask.clone();
    }
  }

  bool getTableLocation(LocateTable::Request& req, LocateTable::Response& res)
  {
    if ( have_depth_data_ )
    {
      res.table_centroid = getTablePlane(cur_point_cloud_);
      if (res.table_centroid.pose.position.x == 0.0 &&
          res.table_centroid.pose.position.y == 0.0 &&
          res.table_centroid.pose.position.z == 0.0)
      {
        ROS_ERROR_STREAM("No plane found, leaving");
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

  void trackerGoalCallback(const tabletop_pushing::SegTrackGoalConstPtr &goal)
  {
    ROS_INFO_STREAM("Tracker callback.");
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
      track_server_.setPreempted();
    }
  }

  void startTracker()
  {
    tracker_initialized_ = false;
    tracking_ = true;

  }
  void stopTracker()
  {
    tracker_initialized_ = false;
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

      ROS_INFO_STREAM("Initializing tracker.");
      table_centroid_ = getTablePlane(cloud);
      if (table_centroid_.pose.position.x == 0.0 &&
          table_centroid_.pose.position.y == 0.0 &&
          table_centroid_.pose.position.z == 0.0)
      {
        ROS_ERROR_STREAM("No plane found!");
      }

      tracker_initialized_ = true;
      tracker_count_ = 0;
      cv::Mat empty_motion(color_frame.rows, color_frame.cols, CV_8UC1,
                           cv::Scalar(0));
      return empty_motion;
    }
#ifndef FAKE_SEGMENT
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
    // Perform graphcut for motion detection
    cv::Mat cut = mgc_(color_frame_f, depth_frame, flow_outs[0], flow_outs[1],
                       flow_outs[2], cur_workspace_mask_, hands_and_arms);
    // Perform graphcut for arm localization
    cv::Mat arm_cut(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
    if (hands_and_arms[0].size() > 0 || hands_and_arms[1].size() > 0)
    {
      arm_cut = mgc_.segmentArmFromMoving(color_frame_f, depth_frame,
                                          cur_workspace_mask_, hands_and_arms,
                                          min_arm_x, max_arm_x, min_arm_y,
                                          max_arm_y);
    }
    cv::Mat cleaned_cut(cut.size(), CV_8UC1);
    cut.convertTo(cleaned_cut, CV_8UC1, 255, 0);
#ifdef REMOVE_SMALL_BLOBS
    // Remove blobs smaller than min_contour_size_
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(cleaned_cut, contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_NONE);
    for (unsigned int i = 0; i < contours.size(); ++i)
    {
      cv::Moments m;
      cv::Mat pt_mat = cv::Mat(contours[i]);
      m = cv::moments(pt_mat);
      if (m.m00 < min_contour_size_)
      {
        for (unsigned int j = 0; j < contours[i].size(); ++j)
        {
          cleaned_cut.at<uchar>(contours[i][j].y, contours[i][j].x) = 0;
        }
      }
    }
#endif // REMOVE_SMALL_BLOBS
#else // FAKE_SEGMENT
    cv::Mat cleaned_cut(color_frame.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat arm_cut(color_frame.size(), CV_32FC1, cv::Scalar(0.0));
#endif // FAKE_SEGMENT
    // Publish the moving region stuff
    cv_bridge::CvImage motion_mask_msg;
    motion_mask_msg.image = cleaned_cut;
    motion_mask_msg.header = cur_camera_header_;
    motion_mask_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    motion_mask_pub_.publish(motion_mask_msg.toImageMsg());

    // Also publish color version
    cv::Mat moving_regions_img;
    color_frame.copyTo(moving_regions_img, cleaned_cut);
    cv_bridge::CvImage motion_img_msg;
    cv::Mat motion_img_send(cleaned_cut.size(), CV_8UC3);
    moving_regions_img.convertTo(motion_img_send, CV_8UC3, 1.0, 0);
    motion_img_msg.image = motion_img_send;
    motion_img_msg.header = cur_camera_header_;
    motion_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    motion_img_pub_.publish(motion_img_msg.toImageMsg());

    cv::Mat cleaned_arm_cut(arm_cut.size(), CV_8UC1);
    arm_cut.convertTo(cleaned_arm_cut, CV_8UC1, 255, 0);
    cv::Mat arm_regions_img;
    color_frame.copyTo(arm_regions_img, cleaned_arm_cut);
    cv::Mat not_arm_move = cleaned_cut - cleaned_arm_cut;
    cv::Mat not_arm_move_color;
    color_frame.copyTo(not_arm_move_color, not_arm_move);
#ifdef WRITE_INPUT_TO_DISK
    std::stringstream input_out_name;
    input_out_name << base_output_path_ << "input" << tracker_count_ << ".tiff";
    if (tracker_count_ < 2)
      ROS_INFO_STREAM("writing input to " << input_out_name.str());
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
#endif // WRITE_ARM_CUT_TO_DISK
#ifdef DISPLAY_INPUT_COLOR
    std::vector<cv::Mat> hsv;
    cv::split(color_frame_f, hsv);
    cv::imshow("hue", hsv[0]);
    cv::imshow("saturation", hsv[1]);
    // cv::imshow("intensity", hsv[2]);
    // cv::imshow("input_color", color_frame);
#endif // DISPLAY_INPUT_COLOR
#ifdef DISPLAY_ARM_CIRCLES
    cv::Mat arms_img(color_frame.size(), CV_8UC3);
    arms_img = color_frame.clone();
    for (unsigned int i = 0; i < hands_and_arms.size(); ++i)
    {
      for (unsigned int j = 0; j < hands_and_arms[i].size(); ++j)
      {
        cv::circle(arms_img, hands_and_arms[i][j], 2, cv::Scalar(0,255,0));
      }
    }
    cv::imshow("arms", arms_img);
#endif
#ifdef DISPLAY_INPUT_DEPTH
    double depth_max = 1.0;
    cv::minMaxLoc(depth_frame, NULL, &depth_max);
    cv::Mat depth_display = depth_frame.clone();
    depth_display /= depth_max;
    cv::imshow("input_depth", depth_display);
#endif // DISPLAY_INPUT_DEPTH
#ifdef DISPLAY_WORKSPACE_MASK
    cv::imshow("workspace_mask", cur_workspace_mask_);
#endif // DISPLAY_WORKSPACE_MASK
#ifdef DISPLAY_OPTICAL_FLOW
    displayOpticalFlow(color_frame, flow_outs[0], flow_outs[1],
                       mgc_.magnitude_thresh_);
#endif // DISPLAY_OPTICAL_FLOW

#ifdef DISPLAY_GRAPHCUT
    // cv::imshow("Cut", cut);
    // cv::imshow("Cleaned Cut", cleaned_cut);
    cv::imshow("moving_regions", moving_regions_img);
    cv::imshow("arm_cut", arm_regions_img);
    cv::imshow("not_arm_move", not_arm_move_color);
#endif // DISPLAY_GRAPHCUT

#if defined DISPLAY_INPUT_COLOR || defined DISPLAY_INPUT_DEPTH || defined DISPLAY_OPTICAL_FLOW || defined DISPLAY_GRAPHCUT || defined DISPLAY_WORKSPACE_MASK || defined DISPLAY_OPT_FLOW_INTERNALS || defined DISPLAY_GRAPHCUT || defined VISUALIZE_GRAPH_WEIGHTS || defined VISUALIZE_ARM_GRAPH_WEIGHTS || defined DISPLAY_ARM_CIRCLES
    cv::waitKey(display_wait_ms_);
#endif // Any display defined
    ++tracker_count_;
    return cleaned_cut;
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
    // Left arm
    cv::Point l0 = projectJointOriginIntoImage(img_header, "l_gripper_tool_frame");
    cv::Point l1 = projectJointOriginIntoImage(img_header, "l_wrist_flex_link");
    cv::Point l2 = projectJointOriginIntoImage(img_header, "l_forearm_roll_link");
    cv::Point l3 = projectJointOriginIntoImage(img_header, "l_upper_arm_roll_link");
    arm_locs.l_chain.push_back(l0);
    arm_locs.l_chain.push_back(l1);
    arm_locs.l_chain.push_back(l2);
    arm_locs.l_chain.push_back(l3);

    // Right arm
    cv::Point r0 = projectJointOriginIntoImage(img_header, "r_gripper_tool_frame");
    cv::Point r1 = projectJointOriginIntoImage(img_header, "r_wrist_flex_link");
    cv::Point r2 = projectJointOriginIntoImage(img_header, "r_forearm_roll_link");
    cv::Point r3 = projectJointOriginIntoImage(img_header, "r_upper_arm_roll_link");
    arm_locs.r_chain.push_back(r0);
    arm_locs.r_chain.push_back(r1);
    arm_locs.r_chain.push_back(r2);
    arm_locs.r_chain.push_back(r3);

    // Keep track of min and max values
    min_x = 10000;
    max_x = 0;
    min_y = 10000;
    max_y = 0;
    arm_locs.l_hand_on = getLineValues(l0, l1, arm_locs.hands, frame_size,
                                       min_x, max_x, min_y, max_y);
    arm_locs.l_arm_on = getLineValues(l1, l2, arm_locs.arms, frame_size,
                                      min_x, max_x, min_y, max_y);
    arm_locs.l_arm_on = (getLineValues(l2, l3, arm_locs.arms, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.l_arm_on);
    arm_locs.r_hand_on = getLineValues(r0, r1, arm_locs.hands, frame_size,
                                       min_x, max_x, min_y, max_y);
    arm_locs.r_arm_on = getLineValues(r1, r2, arm_locs.arms, frame_size,
                                      min_x, max_x, min_y, max_y);
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
    return projectPointIntoImage(joint_origin, img_header.frame_id);
  }

  void drawTableHullOnImage(pcl::PointCloud<pcl::PointXYZ>& hull,
                            PoseStamped cent)
  {
    cv::Mat hull_display = cur_color_frame_.clone();
    cv::Point p_prev;
    for (unsigned int i = 0; i < hull.points.size(); ++i)
    {
      PointStamped h;
      h.header = hull.header;
      h.point.x = hull.points[i].x;
      h.point.y = hull.points[i].y;
      h.point.z = hull.points[i].z;
      ROS_INFO_STREAM("3D Point is at: (" << h.point.x << ", "
                      << h.point.y << ", " << h.point.z << ")");
      cv::Point p = projectPointIntoImage(h, cur_camera_header_.frame_id);
      ROS_INFO_STREAM("2D Point is at: (" << p.x << ", " << p.y << ")");
      cv::circle(hull_display, p, 2, cv::Scalar(0,255,0));
      if (i > 0 && p.x > 0 && p.y > 0 && p_prev.x > 0 && p_prev.y > 0 &&
          p.x < hull_display.cols && p.y < hull_display.rows &&
          p_prev.x < hull_display.cols && p_prev.y < hull_display.rows)
      {
        cv::line(hull_display, p, p_prev, cv::Scalar(0,255,0));
      }
      p_prev = p;
    }
    // cv::Point cent_img = projectPointIntoImage(cent,
    //                                            cur_camera_header_.frame_id);
    // cv::circle(hull_display, cent_img, 5, cv::Scalar(0,0,255));
    cv::imshow("table_image", hull_display);
    cv::waitKey(5);
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

  PoseStamped getTablePlane(XYZPointCloud& cloud)
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
    pcl::PointCloud<pcl::PointXYZ> cloud_hull;
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(boost::make_shared<
                       pcl::PointCloud<pcl::PointXYZ> >(plane_cloud));
    // hull.setAlpha(0.1);
    hull.reconstruct(cloud_hull);
    ROS_INFO_STREAM("Convex hull has: " << cloud_hull.points.size()
                    << " points");
    cloud_hull.header = cloud.header;
    // drawTableHullOnImage(cloud_hull, p);
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
  image_transport::ImageTransport it_;
  sensor_msgs::CameraInfo cam_info_;
  image_transport::Publisher motion_img_pub_;
  image_transport::Publisher motion_mask_pub_;
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
  cv::Mat prev_motion_mask_;
  std_msgs::Header cur_camera_header_;
  std_msgs::Header prev_camera_header_;
  XYZPointCloud cur_point_cloud_;
  LKFlowReliable lkflow_;
  MotionGraphcut mgc_;
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
  double min_table_z_;
  double max_table_z_;
  double eigen_ratio_;
  int num_downsamples_;
  std::string workspace_frame_;
  PoseStamped table_centroid_;
  bool tracking_;
  bool tracker_initialized_;
  cv::Rect roi_;
  std::string arm_data_path_;
  std::string cam_info_topic_;
  int min_contour_size_;
  int tracker_count_;
  std::string base_output_path_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  TabletopPushingPerceptionNode perception_node(n);
  perception_node.spin();

  return 0;
}

