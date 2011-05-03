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

#ifndef color_cell_h_DEFINED
#define color_cell_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "abstract_feature.h"
#include <vector>

namespace cpl_visual_features
{
class ColorCell : public AbstractFeature<std::vector<float> >
{
 public:
  ColorCell(int num_scales=4) : num_scales_(num_scales)
  {
  }

  virtual void operator()(cv::Mat& patch, cv::Rect& window)
  {
    std::vector<cv::Mat> channels;
    cv::split(patch, channels);
    std::vector<float> descriptor;

    for (int scale = 1; scale <= num_scales_; ++scale)
    {
      // Examine cells at each scale
      int cell_height = patch.rows / float(scale);
      int cell_width = patch.cols / float(scale);

      for (int cell_r = 0, vert_count = 0;
           cell_r < patch.rows && vert_count < scale;
           cell_r += cell_height, vert_count++)
      {
        for (int cell_c = 0, horz_count = 0;
             cell_c < patch.cols && horz_count < scale;
             cell_c += cell_width, horz_count++)
        {
          float r_avg = 0;
          float g_avg = 0;
          float b_avg = 0;

          // Compute the average value for each color channel in the cell
          for (int r = cell_r; r < cell_r + cell_height; r++)
          {
            for (int c = cell_c; c < cell_c + cell_height; c++)
            {
              b_avg += channels[0].at<uchar>(r,c);
              g_avg += channels[1].at<uchar>(r,c);
              r_avg += channels[2].at<uchar>(r,c);
            }
          }
          r_avg /= float(cell_width*cell_height);
          g_avg /= float(cell_width*cell_height);
          b_avg /= float(cell_width*cell_height);

          // Add the 6 color descriptors for this cell
          descriptor.push_back(r_avg);
          descriptor.push_back(g_avg);
          descriptor.push_back(b_avg);
          descriptor.push_back((r_avg - g_avg)/float(r_avg + g_avg + b_avg));
          descriptor.push_back((b_avg - (r_avg + g_avg)/2.0)/
                               float(r_avg + g_avg + b_avg));
          descriptor.push_back(float(r_avg + g_avg + b_avg)/(255*3));
        }
      }
    }

    descriptor_ = descriptor;
  }

  virtual std::vector<float> getDescriptor() const
  {
    return descriptor_;
  }

 protected:
  std::vector<float> descriptor_;
  int num_scales_;
};
}
#endif // color_cell_h_DEFINED
