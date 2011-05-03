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

#ifndef edge_density_h_DEFINED
#define edge_density_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "abstract_feature.h"
#include <vector>
#include <iostream>

namespace cpl_visual_features
{
class EdgeDensity : public AbstractFeature<float>
{
 public:
  CannyEdges() : thresh1_(2100.0), thresh2_(2300.0)
  {
  }

  virtual void operator()(cv::Mat& patch, cv::Rect& window)
  {
    // TODO: Should actually shrink, based on the window, don't use the whole
    // thing...
    cv::Mat edges(patch.size(), CV_8UC1);
    cv::Mat patch_bw(patch.size(), CV_8UC1);

    cv::cvtColor(patch, patch_bw, CV_BGR2GRAY);
    cv::Canny(patch_bw, edges, thresh1_, thresh2_, 5);

    float sum = 0;
    for (int r = 0; r < patch.rows; ++r)
    {
      for (int c = 0; c < patch.cols; ++c)
      {
        if (edge.at<uchar>(r,c) > 0)
          sum += 1;
      }
    }
    float perimeter = window.width*2 + window.height*2;
    descriptor_ = sum / perimeter;
  }

  virtual float getDescriptor() const
  {
    return descriptor_;
  }

 protected:
  float descriptor_;
  double thresh1_;
  double thresh2_;
};
}
#endif // edge_density_h_DEFINED
