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

#ifndef normalized_sum_h_DEFINED
#define normalized_sum_h_DEFINED

#include <opencv2/core/core.hpp>
#include "abstract_feature.h"
namespace cpl_visual_features
{
class NormalizedSum : public AbstractFeature<float>
{
 public:
  NormalizedSum() :
      max_sum_(-1.0), max_loc_(0, 0, 0, 0), descriptor_(0.0f)
  {
  }

  virtual void operator()(cv::Mat& img, cv::Rect& window)
  {
    float area_sum = 0.0;
    for (int i = 0; i < img.rows; ++i)
    {
      for (int j = 0; j < img.cols; ++j)
      {
        area_sum += img.at<uchar>(i,j);
      }
    }

    area_sum /= (img.cols*img.rows);

    if (area_sum > max_sum_)
    {
      max_sum_ = area_sum;
      max_loc_.x = window.x;
      max_loc_.y = window.y;
      max_loc_.height = window.height;
      max_loc_.width = window.width;
    }

    descriptor_ = area_sum;
  }

  float getMax() const { return max_sum_; }
  cv::Rect getMaxLoc() const { return max_loc_; }
  void resetMax()
  {
    max_sum_ = -1.0;
    max_loc_.x = 0.0;
    max_loc_.y = 0.0;
    max_loc_.height = 0.0;
    max_loc_.width = 0.0;
  }

  virtual float getDescriptor() const
  {
    return descriptor_;
  }
 protected:
  float max_sum_;
  cv::Rect max_loc_;
  float descriptor_;
};
}
#endif // normalized_sum_h_DEFINED
