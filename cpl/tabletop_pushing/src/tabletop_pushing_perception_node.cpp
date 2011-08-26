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
#include <utility>
#include <math.h>

// Debugging IFDEFS
// #define DISPLAY_INPUT_COLOR 1
// #define DISPLAY_INPUT_DEPTH 1
// #define DISPLAY_WORKSPACE_MASK 1
// #define DISPLAY_MOTION_PROBS 1
// #define DISPLAY_MEANS 1
// #define DISPLAY_VARS 1
// #define DISPLAY_INTERMEDIATE_PROBS 1
// #define DISPLAY_OPTICAL_FLOW 1
// #define DISPLAY_OPT_FLOW_INTERNALS 1
// #define DISPLAY_GRAPHCUT 1
// #define VISUALIZE_GRAPH_WEIGHTS 1

// Functional IFDEFS
// #define MULTISCALE_OPTICAL_FLOW 1

using tabletop_pushing::PushPose;
using tabletop_pushing::LocateTable;
using geometry_msgs::PoseStamped;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2> MySyncPolicy;
typedef Graph<float, float, float> GraphType;

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
    color_frame.convertTo(bgr_frame, CV_32FC3, 1.0/255, 0);

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

#ifdef DISPLAY_MEANS
    cv::imshow("means", means);
    cv::imshow("d_means", d_means);
#endif // DISPLAY_MEANS

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

#ifdef DISPLAY_VARS
    cv::imshow("vars", vars);
    cv::imshow("d_vars", d_vars);
#endif // DISPLAY_VARS

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
        float prob_d = 1.0 - p_x_gaussian_kenney(x_d, mu_d, var_d);
        if (prob_d > 1.0)
        {
          d_probs_greater++;
        }
        else if (prob_d < 0.0)
        {
          d_probs_lesser++;
        }
        d_motion_probs_.at<float>(r,c) = prob_d;
        // motion_probs_.at<cv::Vec4f>(r,c)[3]*= prob_d;
        motion_probs_.at<cv::Vec4f>(r,c)[3]= max(prob_d,
                                                 motion_probs_.at<cv::Vec4f>(r,c)[3]);
      }
    }

    // TODO: Merge these into a single image?
    std::vector<cv::Mat> motions;
    motions.push_back(motion_probs_);
    motions.push_back(d_motion_probs_);
    return motions;
  }

  cv::Vec4f p_x_gaussian(cv::Vec3f x, cv::Vec3f mu, cv::Vec3f var)
  {
    cv::Vec4f p_x;
    p_x[0] = p_x_gaussian_kenney(x[0], mu[0], var[0]);
    p_x[1] = p_x_gaussian_kenney(x[1], mu[1], var[1]);
    p_x[2] = p_x_gaussian_kenney(x[2], mu[2], var[2]);
    p_x[3] = p_x[0]*p_x[1]*p_x[2];
    // p_x[3] = max(p_x[0],max(p_x[1],p_x[2]));
    return p_x;
  }

  float p_x_gaussian_kenney(float x, float mu, float var)
  {
    float mean_diff = abs(x-mu);
    float sigma = sqrt(var);
    float s0 = T_in_*sigma;
    float s1 = T_out_*sigma;

    if (mean_diff <= s0)
    {
      // Don't make things impossible (i.e. depth does not change when things
      // are too close)
      return 0.99;
      // return 1.0;
    }
    if (mean_diff >= s1)
    {
      return 0.01;
    }
    float m = 1.0/(s0-s1);
    return m*(mean_diff-s1);
  }


  float p_x_gaussian_hist(float x, float mu, float var)
  {
    // TODO: Figure out a better way of approximating width
    float x1 = x-0.05;
    float x2 = x+0.05;
    float p_x = (abs(gaussian_pdf(x2, mu, var) - gaussian_pdf(x1, mu, var))*
                 abs(x2-x1));
    return p_x;
  }

  float gaussian_pdf(float x, float mu, float var)
  {
    if (var == 0.0)
    {
      // Don't make things impossible
      if (x == mu)
      {
        return 0.99;
        // return 1.0;
      }
      else
        return 0.01;
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

class LKFlowReliable
{
 public:
  LKFlowReliable(int win_size = 5) :
      win_size_(win_size)
  {

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

    std::vector<cv::Mat> flow_n_scores = baseLKII(cur_bw, prev_bw);
    // std::vector<cv::Mat> flow_n_scores = baseLK(cur_bw, prev_bw);
    return flow_n_scores;
  }

  std::vector<cv::Mat> baseLK(cv::Mat& cur_bw, cv::Mat& prev_bw)
  {
    // Get gradients using sobel
    cv::Mat Ix(cur_bw.size(), CV_32FC1);
    cv::Mat Iy(cur_bw.size(), CV_32FC1);
    cv::Sobel(cur_bw, Ix, Ix.depth(), 1, 0, 3);
    cv::Sobel(cur_bw, Iy, Iy.depth(), 0, 1, 3);
    cv::Mat It = cur_bw - prev_bw;
#ifdef DISPLAY_OPT_FLOW_INTERNALS
    cv::imshow("cur_bw", cur_bw);
    cv::imshow("prev_bw", prev_bw);
    cv::imshow("It", It);
    cv::imshow("Ix", Ix);
    cv::imshow("Iy", Iy);
#endif // DISPLAY_OPT_FLOW_INTERNALS

    int win_radius = win_size_/2;
    cv::Mat flow(cur_bw.size(), CV_32FC2, cv::Scalar(0.0,0.0));
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
        flow.at<cv::Vec2f>(r,c) = uv;
        t_scores.at<float>(r,c) = (sIxx+sIyy)*(sIxx+sIyy)/(det);
      }
    }
    std::vector<cv::Mat> outs;
    outs.push_back(flow);
    outs.push_back(t_scores);
    return outs;
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
    cv::Mat cur_blur(cur_bw.size(), cur_bw.type());
    cv::Mat prev_blur(prev_bw.size(), prev_bw.type());
    float sigma = 1.5;
    int k_size = 3;
    cv::GaussianBlur(cur_bw, cur_blur, cv::Size(k_size, k_size), sigma, sigma);
    cv::GaussianBlur(prev_bw, prev_blur, cv::Size(k_size, k_size), sigma, sigma);
    cv::Mat Ix32(cur_bw.size(), CV_32FC1);
    cv::Mat Iy32(cur_bw.size(), CV_32FC1);
    cv::Sobel(cur_blur, Ix32, Ix32.depth(), 1, 0, 3);
    cv::Sobel(cur_blur, Iy32, Iy32.depth(), 0, 1, 3);
    cv::Mat Ix(cur_bw.size(), CV_64FC1);
    cv::Mat Iy(cur_bw.size(), CV_64FC1);

    cv::Mat It32 = cur_blur - prev_blur;
    cv::Mat It(It32.size(), CV_64FC1);
    Ix32.convertTo(Ix, CV_64FC1);
    Iy32.convertTo(Iy, CV_64FC1);
    It32.convertTo(It, CV_64FC1);
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
    cv::imshow("cur_bw", cur_bw);
    cv::imshow("prev_bw", prev_bw);
    cv::imshow("It", It);
    cv::imshow("Ix", Ix);
    cv::imshow("Iy", Iy);
#endif // DISPLAY_OPT_FLOW_INTERNALS

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

  int win_size_;
};

class BlobTracker
{
};

class MotionGraphcut
{
 public:
  MotionGraphcut(double eigen_ratio = 5.0f, double w_f = 3.0f, double w_b = 2.0f,
                 double w_n_f = 0.01f, double w_n_b = 0.01f, double w_w_b = 5.0f,
                 double w_u_f = 0.1f, double w_u_b = 0.1f,
                 double magnitude_thresh=0.1) :
      w_f_(w_f), w_b_(w_b), w_n_f_(w_n_f), w_n_b_(w_n_b),  w_w_b_(w_w_b),
      w_u_f_(w_u_f), w_u_b_(w_u_b), magnitude_thresh_(magnitude_thresh)
  {
    setEigenRatio(eigen_ratio);
  }

  virtual ~MotionGraphcut()
  {
  }

  cv::Mat operator()(cv::Mat& color_frame, cv::Mat& depth_frame,
                     cv::Mat& u, cv::Mat& v, cv::Mat flow_scores,
                     cv::Mat& workspace_mask)
  {
    const int R = color_frame.rows;
    const int C = color_frame.cols;
    int num_nodes = R*C;
    int num_edges = ((C-1)*3+1)*(R-1)+(C-1);
    g = new GraphType(num_nodes, num_edges);

#ifdef VISUALIZE_GRAPH_WEIGHTS
    cv::Mat fg_weights(color_frame.size(), CV_32FC1);
    cv::Mat bg_weights(color_frame.size(), CV_32FC1);
#endif // VISUALIZE_GRAPH_WEIGHTS

    for (int r = 0; r < R; ++r)
    {
      for (int c = 0; c < C; ++c)
      {
        g->add_node();
        float magnitude = u.at<float>(r,c) + v.at<float>(r,c) ;

        // Check if we are hardcoding this spot to background
        if (workspace_mask.at<uchar>(r,c) == 0)
        {
          g->add_tweights(r*C+c, /*capacities*/ 0.0, w_w_b_);

#ifdef VISUALIZE_GRAPH_WEIGHTS
          fg_weights.at<float>(r,c) = 0.0;
          bg_weights.at<float>(r,c) = w_w_b_;
#endif // VISUALIZE_GRAPH_WEIGHTS
        }
        else if (flow_scores.at<float>(r,c) < corner_thresh_)
        {

          if (magnitude > magnitude_thresh_)
          {
            g->add_tweights(r*C+c, /*capacities*/ w_f_, w_n_f_);

#ifdef VISUALIZE_GRAPH_WEIGHTS
            fg_weights.at<float>(r,c) = w_f_;
            bg_weights.at<float>(r,c) = w_n_f_;
#endif // VISUALIZE_GRAPH_WEIGHTS
          }
//           else if (magnitude > 0.0)
//           {
//             g->add_tweights(r*C+c, /*capacities*/ w_u_f_, w_u_b_);
// #ifdef VISUALIZE_GRAPH_WEIGHTS
//             fg_weights.at<float>(r,c) = w_u_f_;
//             bg_weights.at<float>(r,c) = w_u_b_;
// #endif // VISUALIZE_GRAPH_WEIGHTS
//           }
          else
          {
            g->add_tweights(r*C+c, /*capacities*/ w_n_b_, w_b_);

#ifdef VISUALIZE_GRAPH_WEIGHTS
            fg_weights.at<float>(r,c) = w_n_b_;
            bg_weights.at<float>(r,c) = w_b_;
#endif // VISUALIZE_GRAPH_WEIGHTS
          }
        }
        else
        {
          g->add_tweights(r*C+c, /*capacities*/ w_u_f_, w_u_b_);
#ifdef VISUALIZE_GRAPH_WEIGHTS
          fg_weights.at<float>(r,c) = w_u_f_;
          bg_weights.at<float>(r,c) = w_u_b_;
#endif // VISUALIZE_GRAPH_WEIGHTS
        }

        // Connect node to previous ones
        if (c > 0)
        {
          // Add left-link
          float w_l = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r,c-1),
                                    depth_frame.at<float>(r,c-1));
          g->add_edge(r*C+c, r*C+c-1, /*capacities*/ w_l, w_l);
        }

        if (r > 0)
        {
          // Add up-link
          float w_u = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                    depth_frame.at<float>(r,c),
                                    color_frame.at<cv::Vec3f>(r-1,c),
                                    depth_frame.at<float>(r-1,c));
          g->add_edge(r*C+c, (r-1)*C+c, /*capacities*/ w_u, w_u);
          // Add up-left-link
          if (c > 0)
          {
            float w_ul = getEdgeWeight(color_frame.at<cv::Vec3f>(r,c),
                                       depth_frame.at<float>(r,c),
                                       color_frame.at<cv::Vec3f>(r-1,c-1),
                                       depth_frame.at<float>(r-1,c-1));
            g->add_edge(r*C+c, (r-1)*C+c-1, /*capacities*/ w_ul, w_ul);
          }
        }
      }
    }

#ifdef VISUALIZE_GRAPH_WEIGHTS
    cv::imshow("fg_weights", fg_weights);
    cv::imshow("bg_weights", bg_weights);
#endif // VISUALIZE_GRAPH_WEIGHTS

    int flow = g->maxflow(false);

    // Convert output into image
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
    delete g;
    return segs;
  }

  float getEdgeWeight(cv::Vec3f c0, float d0, cv::Vec3f c1, float d1)
  {
    cv::Vec3f c_d = c0-c1;
    float w_d = d0-d1;
    // float w_c = sqrt(c_d[0]*c_d[0]+c_d[1]*c_d[1]+c_d[2]*c_d[2] + w_d*w_d);
    // float w_c = sqrt(c_d[0]*c_d[0] + c_d[1]*c_d[1] + w_d*w_d);
    float w_c = sqrt(c_d[0]*c_d[0] + w_d*w_d);
    return w_c;
  }

  void setEigenRatio(double eigen_ratio)
  {
    corner_thresh_ = (eigen_ratio+1)*(eigen_ratio+1)/eigen_ratio;
  }

 protected:
  GraphType *g;
  double corner_thresh_;

 public:
  double w_f_;
  double w_b_;
  double w_n_f_;
  double w_n_b_;
  double w_w_b_;
  double w_u_f_;
  double w_u_b_;
  double magnitude_thresh_;
};

class RobustBlobTracking
{
 public:
  RobustBlobTracking()
  {
  }
};

class PR2ArmDetector
{
 public:
  PR2ArmDetector() : have_model_(false)
  {
    means_.clear();
    vars_.clear();
    weights_.clear();
  }

  void setColorModel(std::vector<cv::Vec3f> means,
                     std::vector<cv::Vec3f> vars,
                     std::vector<float> weights)
  {
    have_model_ = true;
    means_ = means;
    vars_ = vars;
    weights_ = weights;
  }

  float pixelArmProbability(cv::Vec3f pixel_color)
  {
    if (not have_model_) return 0.0;
    // TODO: Test pixel vs color model; assumes HSV
    cv::Vec3f diff = means_[0] - pixel_color;
    float prob = 1.0 - (abs(diff[0]) + abs(diff[1]) + abs(diff[2])) / 3.0;
    return prob;
  }

  cv::Mat imageArmProbability(cv::Mat& arm_img)
  {
    cv::Mat prob_img(arm_img.size(), CV_32FC1);
    for(int r = 0; r < arm_img.rows; ++r)
    {
      cv::Vec3f* arm_row = arm_img.ptr<cv::Vec3f>(r);
      float* prob_row = prob_img.ptr<float>(r);
      for (int c = 0; c < arm_img.cols; ++c)
      {
        prob_row[c] = pixelArmProbability(arm_row[c]);
      }
    }
    return prob_img;
  }

 protected:
  bool have_model_;
  std::vector<cv::Vec3f> means_;
  std::vector<cv::Vec3f> vars_;
  std::vector<float> weights_;
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
      it_(n),
      track_server_(n, "seg_track_action",
                    boost::bind(&TabletopPushingPerceptionNode::trackerGoalCallback,
                                this, _1),
                    false),
      tf_(), /*motion_probs_(5),*/ have_depth_data_(false), tracking_(false),
      tracker_initialized_(false), roi_(0,0,640,480)
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
    n_private.param("erosion_size", erosion_size_, 3);
    n_private.param("num_downsamples", num_downsamples_, 2);

    // Graphcut weights
    n_private.param("mgc_w_f", mgc_.w_f_, 3.0);
    n_private.param("mgc_w_b", mgc_.w_b_, 2.0);
    n_private.param("mgc_w_n_f", mgc_.w_n_f_, 0.01);
    n_private.param("mgc_w_n_b", mgc_.w_n_b_, 0.01);
    n_private.param("mgc_w_w_b", mgc_.w_w_b_, 5.0);
    n_private.param("mgc_w_u_f", mgc_.w_u_f_, 0.1);
    n_private.param("mgc_w_u_b", mgc_.w_u_b_, 0.1);

    // Lucas Kanade params
    double eigen_ratio = 0.5;
    n_private.param("mgc_eigen_ratio", eigen_ratio, 0.5);
    mgc_.setEigenRatio(eigen_ratio);
    n_private.param("mgc_magnitude_thresh", mgc_.magnitude_thresh_, 0.1);
    n_private.param("lk_win_size", lkflow_.win_size_, 5);

    // int num_hist_frames = 5;
    // n_private.param("num_hist_frames", num_hist_frames, 5);
    // motion_probs_.setNumHistFrames(num_hist_frames);
    // double T_in = 3;
    // double T_out = 5;
    // n_private.param("T_in", T_in, 3.0);
    // n_private.param("T_out", T_out, 5.0);
    // motion_probs_.setT_inT_out(T_in, T_out);

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

    // Update the current versions
    cur_color_frame_ = color_frame_down.clone();
    cur_depth_frame_ = depth_frame_down.clone();
    cur_workspace_mask_ = workspace_mask_down.clone();
    cur_point_cloud_ = cloud;
    have_depth_data_ = true;

    // Started via actionlib call
    if (tracking_)
      trackRegions(cur_color_frame_, cur_depth_frame_,
                   prev_color_frame_, prev_depth_frame_,
                   cur_point_cloud_);
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
  void trackRegions(cv::Mat& color_frame, cv::Mat& depth_frame,
                    cv::Mat& prev_color_frame, cv::Mat& prev_depth_frame,
                    XYZPointCloud& cloud)
  {
    if (!tracker_initialized_)
    {
      // motion_probs_.initialize(color_frame, depth_frame);
      ROS_INFO_STREAM("Initializing tracker.");
      table_centroid_ = getTablePlane(cloud);
      if (table_centroid_.pose.position.x == 0.0 &&
          table_centroid_.pose.position.y == 0.0 &&
          table_centroid_.pose.position.z == 0.0)
      {
        ROS_ERROR_STREAM("No plane found!");
      }

      tracker_initialized_ = true;
      return;
    }

    cv::Mat color_frame_hsv(color_frame.size(), color_frame.type());
    cv::cvtColor(color_frame, color_frame_hsv, CV_BGR2HSV);

    std::vector<cv::Mat> flow_outs = lkflow_(color_frame, prev_color_frame);
    std::vector<cv::Mat> flows;
    cv::split(flow_outs[0], flows);

    cv::Mat color_frame_f(color_frame_hsv.size(), CV_32FC3);
    color_frame_hsv.convertTo(color_frame_f, CV_32FC3, 1.0/255, 0);
    cv::Mat cut = mgc_(color_frame_f, depth_frame, flows[0], flows[1],
                       flow_outs[1], cur_workspace_mask_);

    // Perform close morphology
    cv::Mat element(erosion_size_, erosion_size_, CV_32FC1, cv::Scalar(1.0));
    cv::Mat eroded_cut(cut.size(), cut.type());
    cv::Mat dialated_cut(cut.size(), cut.type());
    cv::dilate(cut, dialated_cut, element);
    cv::erode(dialated_cut, eroded_cut, element);

    cv_bridge::CvImage motion_mask_msg;
    cv::Mat motion_mask_send(cut.size(), CV_8UC1);
    eroded_cut.convertTo(motion_mask_send, CV_8UC1, 255, 0);
    motion_mask_msg.image = motion_mask_send;
    motion_mask_msg.header.frame_id = "/openni_rgb_optical_frame";
    motion_mask_msg.header.stamp = ros::Time::now();
    motion_mask_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    motion_mask_pub_.publish(motion_mask_msg.toImageMsg());

    // Also publish color version
    cv::Mat moving_regions_img;
    color_frame.copyTo(moving_regions_img, motion_mask_send);
    cv_bridge::CvImage motion_img_msg;
    cv::Mat motion_img_send(eroded_cut.size(), CV_8UC3);
    moving_regions_img.convertTo(motion_img_send, CV_8UC3, 1.0, 0);
    motion_img_msg.image = motion_img_send;
    motion_img_msg.header.frame_id = "/openni_rgb_optical_frame";
    motion_img_msg.header.stamp = motion_mask_msg.header.stamp;
    motion_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    motion_img_pub_.publish(motion_img_msg.toImageMsg());

#ifdef DISPLAY_INPUT_COLOR
    std::vector<cv::Mat> hsv;
    cv::split(color_frame_hsv, hsv);
    cv::imshow("hue", hsv[0]);
    cv::imshow("saturation", hsv[1]);
    cv::imshow("intensity", hsv[2]);
    cv::imshow("input_color", color_frame);
#endif // DISPLAY_INPUT_COLOR
#ifdef DISPLAY_INPUT_DEPTH
    cv::imshow("input_depth", depth_frame);
#endif // DISPLAY_INPUT_DEPTH
#ifdef DISPLAY_WORKSPACE_MASK
    cv::imshow("workspace_mask", cur_workspace_mask_);
#endif // DISPLAY_WORKSPACE_MASK

#ifdef DISPLAY_OPTICAL_FLOW
    cv::Mat flow_thresh_disp_img(color_frame.size(), CV_8UC3);
    flow_thresh_disp_img = color_frame.clone();

    float corner_thresh = (eigen_ratio_+1)*(eigen_ratio_+1)/eigen_ratio_;
    for (int r = 0; r < flow_thresh_disp_img.rows; ++r)
    {
      for (int c = 0; c < flow_thresh_disp_img.cols; ++c)
      {
        cv::Vec2f uv = flow_outs[0].at<cv::Vec2f>(r,c);
        if (abs(uv[0])+abs(uv[1]) > mgc_.magnitude_thresh_ &&
            flow_outs[1].at<float>(r,c) < corner_thresh)
        {
          cv::line(flow_thresh_disp_img, cv::Point(c,r),
                   cv::Point(c-uv[0], r-uv[1]), cv::Scalar(0,255,0));
        }
      }
    }
    cv::imshow("flow_disp", flow_thresh_disp_img);
    // cv::imshow("u", flows[0]);
    // cv::imshow("v", flows[1]);
#endif // DISPLAY_OPTICAL_FLOW

#ifdef DISPLAY_GRAPHCUT
    cv::imshow("Cut", cut);
    cv::imshow("Eroded Cut", eroded_cut);
    // cv::imshow("Upper Cut", upper_cut);
    cv::imshow("moving_regions", moving_regions_img);
#endif // DISPLAY_GRAPHCUT

#if defined DISPLAY_INPUT_COLOR || defined DISPLAY_INPUT_DEPTH || defined DISPLAY_OPTICAL_FLOW || defined DISPLAY_GRAPHCUT || defined DISPLAY_GRAPHCUT || defined DISPLAY_WORKSPACE_MASK || defined DISPLAY_MOTION_PROBS || defined DISPLAY_MEANS || defined DISPLAY_VARS || defined DISPLAY_INTERMEDIATE_PROBS || defined DISPLAY_OPT_FLOW_INTERNALS || defined DISPLAY_OPT_FLOW_II_INTERNALS || defined DISPLAY_GRAPHCUT || defined VISUALIZE_GRAPH_WEIGHTS
    cv::waitKey(display_wait_ms_);
#endif // Any display defined
  }

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

  cv::Mat downSelect(cv::Mat& data_in)
  {
    // TODO: Check types...
    cv::Mat down(data_in.rows/2, data_in.cols/2, CV_32FC1);
    for (int r = 0; r < down.rows; ++r)
    {
      float* down_row = down.ptr<float>(r);
      float* data_in_row = down.ptr<float>(r*2+1);
      for (int c = 0; c < down.cols; ++c)
      {
        //down.at<float>(r,c) = data_in.at<float>(r*2+1, c*2+1);
        down_row[c] = data_in_row[c*2+1];
      }
    }
    return down;
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
  XYZPointCloud cur_point_cloud_;
  // ProbImageDifferencing motion_probs_;
  LKFlowReliable lkflow_;
  MotionGraphcut mgc_;
  bool have_depth_data_;
  int crop_min_x_;
  int crop_max_x_;
  int crop_min_y_;
  int crop_max_y_;
  int display_wait_ms_;
  int erosion_size_;
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
  double eigen_ratio_;
  int num_downsamples_;
  std::string workspace_frame_;
  PoseStamped table_centroid_;
  bool tracking_;
  bool tracker_initialized_;
  bool detecting_arm_;
  cv::Rect roi_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  TabletopPushingPerceptionNode perception_node(n);
  perception_node.spin();
}
