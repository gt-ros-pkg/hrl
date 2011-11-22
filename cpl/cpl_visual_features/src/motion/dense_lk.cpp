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

#include <cpl_visual_features/motion/dense_lk.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace cpl_visual_features
{

DenseLKFlow::DenseLKFlow(int win_size, int num_levels) :
    win_size_(win_size), max_level_(num_levels-1)
{
  // Create derivative kernels for flow calculation
  cv::getDerivKernels(dy_kernel_, dx_kernel_, 1, 0, CV_SCHARR, true, CV_32F);
  cv::flip(dy_kernel_, dy_kernel_, -1);
  g_kernel_ = cv::getGaussianKernel(3, 2.0, CV_32F);
  optic_g_kernel_ = cv::getGaussianKernel(5, 10.0, CV_32F);
  cv::transpose(dy_kernel_, dx_kernel_);
}

DenseLKFlow::~DenseLKFlow()
{
}

std::vector<cv::Mat> DenseLKFlow::operator()(cv::Mat& cur_frame,
                                             cv::Mat& prev_frame,
                                             bool color_in)
{
  // Convert to grayscale
  if (color_in)
  {
    cv::Mat tmp_bw(cur_frame.size(), CV_8UC1);
    cv::Mat cur_bw(cur_frame.size(), CV_32FC1);
    cv::Mat prev_bw(prev_frame.size(), CV_32FC1);
    cv::cvtColor(cur_frame, tmp_bw, CV_BGR2GRAY);
    tmp_bw.convertTo(cur_bw, CV_32FC1, 1.0/255, 0);
    cv::cvtColor(prev_frame, tmp_bw, CV_BGR2GRAY);
    tmp_bw.convertTo(prev_bw, CV_32FC1, 1.0/255, 0);
    return hierarchy(cur_bw, prev_bw);
  }
  else
  {
    return hierarchy(cur_frame, prev_frame);
  }
}

std::vector<cv::Mat> DenseLKFlow::hierarchy(cv::Mat& f2, cv::Mat& f1)
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

std::vector<cv::Mat> DenseLKFlow::baseLK(cv::Mat& cur_bw, cv::Mat& prev_bw)
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

cv::Mat DenseLKFlow::reduce(cv::Mat& input)
{
  cv::Mat output;// = input.clone();
  cv::pyrDown(input, output);
  return output;
}

cv::Mat DenseLKFlow::expand(cv::Mat& input)
{
  cv::Mat output(input.rows*2, input.cols*2, CV_32FC1);
  cv::pyrUp(input, output, output.size());
  return output;
}

cv::Mat DenseLKFlow::smooth(cv::Mat& input, int n)
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

cv::Mat DenseLKFlow::warp(cv::Mat& i2, cv::Mat& vx, cv::Mat& vy)
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
void DenseLKFlow::setWinSize(int win_size)
{
  win_size_ = win_size;
}

void DenseLKFlow::setNumLevels(int num_levels)
{
  max_level_ = num_levels-1;
}
}
