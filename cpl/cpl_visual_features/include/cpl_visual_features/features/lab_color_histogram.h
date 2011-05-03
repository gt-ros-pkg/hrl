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

#ifndef lab_color_histogram_h_DEFINED
#define lab_color_histogram_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "abstract_feature.h"
#include <iostream>

namespace cpl_visual_features
{
template<int l_bins, int a_bins, int b_bins> class LabColorHistogram :
      public AbstractFeature<std::vector<float> >
{
 public:
  LabColorHistogram()
  {
  }

  virtual void operator()(cv::Mat& patch, cv::Rect& window)
  {
    // First convert to be a 32 bit image
    cv::Mat cvt_patch(patch.rows, patch.cols, CV_32FC3);
    patch.convertTo(cvt_patch, CV_32F, 1.0/255.0);

    // Now convert to be in the Lab color space
    cv::Mat lab_patch(patch.rows, patch.cols, CV_32FC3);
    cv::cvtColor(cvt_patch, lab_patch, CV_BGR2Lab);

    int bins[] = {l_bins, a_bins, b_bins};
    int channels[] = {0,1,2};
    float l_ranges[] = {0,100};
    float a_ranges[] = {-128,128};
    float b_ranges[] = {-128,128};
    const float* ranges[] = {l_ranges, a_ranges, b_ranges};

    cv::MatND patch_desc;

    cv::calcHist(&lab_patch, 1, channels, cv::Mat(), patch_desc, 3,
                 bins, ranges, true, false);

    std::vector<float> desc;
    float color_sum = 0;

    for (int i = 0; i < l_bins; ++i)
    {
      for (int j = 0; j < a_bins; ++j)
      {
        for (int k = 0; k < b_bins; ++k)
        {
          desc.push_back(patch_desc.at<float>(i,j,k));
          color_sum += patch_desc.at<float>(i,j,k);
        }
      }
    }

#ifdef NORMALIZE_RESULTS
    for (unsigned int i = 0; i < desc.size(); ++i)
    {
      desc[i] = desc[i] / color_sum;
    }
#endif // NORMALIZE_RESULTS

    descriptor_ = desc;
  }

  virtual std::vector<float> getDescriptor() const
  {
    return descriptor_;
  }

 protected:
  std::vector<float> descriptor_;
};
}
#endif // lab_color_histogram_h_DEFINED
