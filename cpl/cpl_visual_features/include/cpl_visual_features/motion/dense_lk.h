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

#ifndef dense_lk_h_DEFINED
#define dense_lk_h_DEFINED

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// STL
#include <vector>
#include <cpl_visual_features/motion/flow_types.h>

namespace cpl_visual_features
{
class DenseLKFlow
{
 public:
  DenseLKFlow(int win_size = 5, int num_levels = 4);

  virtual ~DenseLKFlow();

  std::vector<cv::Mat> operator()(cv::Mat& cur_frame, cv::Mat& prev_frame,
                                  bool color_in=true);
  std::vector<cv::Mat> hierarchy(cv::Mat& f2, cv::Mat& f1);
  std::vector<cv::Mat> baseLK(cv::Mat& cur_bw, cv::Mat& prev_bw);
  cv::Mat reduce(cv::Mat& input);
  cv::Mat expand(cv::Mat& input);
  cv::Mat smooth(cv::Mat& input, int n=1);
  cv::Mat warp(cv::Mat& i2, cv::Mat& vx, cv::Mat& vy);
  //
  // Getters and Setters
  //
  void setWinSize(int win_size);

  void setNumLevels(int num_levels);
  double r_thresh_;
 protected:
  int win_size_;
  int max_level_;
  cv::Mat dx_kernel_;
  cv::Mat dy_kernel_;
  cv::Mat g_kernel_;
  cv::Mat optic_g_kernel_;
};

void displayOpticalFlow(cv::Mat& color_frame, cv::Mat& flow_u, cv::Mat& flow_v,
                        float mag_thresh, bool display_uv=false)
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
  if (display_uv)
  {
    cv::imshow("u", flow_u);
    cv::imshow("v", flow_v);
  }
}

}
#endif // dense_lk_h_DEFINED
