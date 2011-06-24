/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Georgia Institute of Technology
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

#ifndef center_surround_h_DEFINED
#define center_surround_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace cpl_visual_features
{
class CenterSurroundMapper
{
 public:
  CenterSurroundMapper(int min_c=2, int max_c=4, int min_delta = 3,
                       int max_delta = 4);

  cv::Mat operator()(cv::Mat& frame, bool use_gradient=false);
  cv::Mat operator()(cv::Mat& frame, cv::Mat& depth_map);
  cv::Mat getSaliencyMap(cv::Mat& frame);
  cv::Mat getIntensityMap(cv::Mat& frame);
  cv::Mat getColorMap(cv::Mat& frame);
  cv::Mat getOrientationMap(cv::Mat& frame);
  cv::Mat mapDifference(cv::Mat& m_c, cv::Mat& m_s, int c, int s);
  cv::Mat mapSum(std::vector<cv::Mat>& maps);
  cv::Mat normalize(cv::Mat& map, float max_val);
  cv::Mat scaleMap(cv::Mat map);
  cv::Mat upSampleResponse(cv::Mat& m_s, int s, cv::Size size0);
  void generateGaborFilters();

 protected:
  int num_scales_;
  int min_c_;
  int max_c_;
  int min_delta_;
  int max_delta_;
  int N_;
  int gabor_size_;
  std::vector<cv::Mat> gabor_filters_;
};
}
#endif // center_surround_h_DEFINED
