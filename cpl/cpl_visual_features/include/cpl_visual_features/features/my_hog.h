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

#ifndef my_hog_h_DEFINED
#define my_hog_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "abstract_feature.h"
#include <vector>
#include <iostream>

namespace cpl_visual_features
{
template <int win_width, int win_height, int block_width, int block_height,
          int x_stride, int y_stride, int cell_width, int cell_height,
          int nbins> class MyHOG
    : public AbstractFeature<std::vector<float> >
{
 public:
  MyHOG() : hog_(cv::Size(win_width, win_height),
                 cv::Size(block_width, block_height),
                 cv::Size(x_stride, y_stride),
                 cv::Size(cell_width, cell_height),
                 nbins)
  {
  }

  virtual void operator()(cv::Mat& img, cv::Rect& window)
  {
    std::vector<float> descriptors;
    if (cv::Rect() == window)
    {
        hog_.compute(img, descriptors);
        std::cout << "Computing HOG on entire image" << std::endl;
    }
    else
    {
      cv::Size win_stride(window.width, window.height);
      cv::Point location(window.x, window.y);
      std::vector<cv::Point> locations(1,location);
      hog_.compute(img, descriptors, win_stride, cv::Size(), locations);
    }

    descriptor_ = descriptors;
  }

  virtual std::vector<float> getDescriptor() const
  {
    return descriptor_;
  }

 public:
  cv::HOGDescriptor hog_;

 protected:
  std::vector<float> descriptor_;
};
}
#endif // my_hog_h_DEFINED
