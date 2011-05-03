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

#ifndef color_histogram_h_DEFINED
#define color_histogram_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "abstract_feature.h"
namespace cpl_visual_features
{
template<int n_bins> class ColorHistogram : public AbstractFeature<cv::MatND>
{
 public:
  ColorHistogram()
  {
  }

  virtual void operator()(cv::Mat& patch, cv::Rect& window)
  {
    int bins[] = {n_bins, n_bins, n_bins};
    int channels[] = {0,1,2};
    float r_ranges[] = {0,256};
    float g_ranges[] = {0,256};
    float b_ranges[] = {0,256};
    const float* ranges[] = {r_ranges, g_ranges, b_ranges};

    cv::MatND patch_desc;

    cv::calcHist(&patch, 1, channels, cv::Mat(), patch_desc, 3, bins, ranges,
                 true, false);
    descriptor_ = patch_desc;
  }

  virtual cv::MatND getDescriptor() const
  {
    return descriptor_;
  }

 protected:
  cv::MatND descriptor_;
};
}
#endif // color_histogram_h_DEFINED
