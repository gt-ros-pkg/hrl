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

#ifndef canny_edges_h_DEFINED
#define canny_edges_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "abstract_feature.h"
#include <vector>
#include <iostream>

// #define CANNY_EDGES_SHOW_IMAGES

namespace cpl_visual_features
{
class CannyEdges : public AbstractFeature<std::vector<int> >
{
 public:
  CannyEdges(int num_orientations = 8) : num_orientations_(num_orientations),
                                         thresh1_(2100.0), thresh2_(2300.0)
  {
  }

  virtual void operator()(cv::Mat& patch, cv::Rect& window)
  {
    cv::Mat patch_bw(patch.size(), CV_8UC1);
    cv::Mat edges(patch.size(), CV_8UC1);
    cv::Mat edges_dx(patch.size(), CV_64FC1);
    cv::Mat edges_dy(patch.size(), CV_64FC1);
    cv::Mat orientations(patch.size(), CV_64FC1, 0.0);

    if (patch.channels() == 3)
    {
      cv::cvtColor(patch, patch_bw, CV_BGR2GRAY);
    }
    else
    {
      patch_bw = patch;
    }

    cv::Canny(patch_bw, edges, thresh1_, thresh2_, 5);

    // Run a Sobel filter on the canny image to get the orientations
    cv::Sobel(edges, edges_dx, edges_dx.depth(), 1, 0, 3);
    cv::Sobel(edges, edges_dy, edges_dy.depth(), 0, 1, 3);

    // Find the orientation at each pixel
    for(int r = 0; r < orientations.rows; ++r)
    {
      for(int c = 0; c < orientations.cols; ++c)
      {
        orientations.at<double>(r,c) = atan2(edges_dy.at<double>(r,c),
                                             edges_dx.at<double>(r,c));
      }
    }

    // Quantize the edges orientations into num_orientations_ unsigned bins
    std::vector<int> descriptor(num_orientations_, 0);

    for(int r = 0; r < orientations.rows; ++r)
    {
      for(int c = 0; c < orientations.cols; ++c)
      {
        // If it is an edge pixel
        if (edges.at<uchar>(r,c) != 0)
        {
          // Increment the correct histogram bin
          for (int i = 0; i < num_orientations_; i++)
          {
            if ( (M_PI/num_orientations_*(i+1)) >=
                 std::abs(orientations.at<double>(r,c)) )
            {
              descriptor[i]++;
              break;
            }
          }
        }
      }
    }

    // Set the class to descriptor to the last extracted one
    descriptor_ = descriptor;

#ifdef CANNY_EDGES_SHOW_IMAGES
    //cv::imshow("patch", patch);
    cv::imshow("edges", edges);
    cv::imshow("edges_dx", edges_dx);
    cv::imshow("edges_dy", edges_dy);
    cv::imshow("orientations", orientations);
    cv::waitKey();
#endif // CANNY_EDGES_SHOW_IMAGES
  }

  virtual std::vector<int> getDescriptor() const
  {
    return descriptor_;
  }

 protected:
  std::vector<int> descriptor_;
  int num_orientations_;
  double thresh1_;
  double thresh2_;
};
}
#endif // canny_edges_h_DEFINED
