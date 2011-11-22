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

#ifndef feature_tracker_h_DEFINED
#define feature_tracker_h_DEFINED

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
// STL
#include <vector>
#include <deque>
#include <cpl_visual_features/motion/flow_types.h>
#include <utility>

namespace cpl_visual_features
{

typedef std::vector<float> Descriptor;
typedef std::vector<Descriptor> Descriptors;
typedef std::vector<cv::KeyPoint> KeyPoints;

class FeatureTracker
{
 public:
  FeatureTracker(std::string name, double hessian_thresh=250, int num_octaves=4,
                 int num_layers=2, bool extended=true, bool upright=false,
                 bool use_fast=false);

  //
  // Main tracking logic functions
  //

  void initTracks(cv::Mat& frame);

  AffineFlowMeasures updateTracksLK(cv::Mat& cur_frame, cv::Mat& prev_frame);

  AffineFlowMeasures updateTracks(const cv::Mat& frame);

  AffineFlowMeasures updateTracks(const cv::Mat& frame, const cv::Mat& mask);

  //
  // Feature Matching Functions
  //
 protected:
  /*
   * SSD
   *
   * @short Computes the squareroot of squared differences
   * @param a First descriptor
   * @param b second descriptor
   * @return value of squareroot of squared differences
   */
  double SSD(Descriptor& a, Descriptor& b);

  /*
   * ratioTest
   *
   * @short Computes the  ratio test described in Lowe 2004
   * @param a Descriptor from the first image to compare
   * @param bList List of descriptors from the second image
   * @param threshold Threshold value for ratioTest comparison
   *
   * @return index of the best match, -1 if no match ratio is less than threshold
   */
  std::pair<int, float> ratioTest(Descriptor& a, std::vector<Descriptor>& bList,
                                  double threshold);

  /**
   * findMatches
   *
   * @param descriptors1 List of descriptors from image 1
   * @param descriptors2 List of descriptors from image 2
   * @param matches1 Indexes of matching points in image 1 (Returned)
   * @param matches2 Indexes of matching points in image 2 (Returned)
   */
  void findMatches(Descriptors& descriptors1, Descriptors& descriptors2,
                   std::vector<int>& matches1, std::vector<int>& matches2,
                   std::vector<float>& scores);

  //
  // Helper Functions
  //

  void updateCurrentDescriptors(const cv::Mat& frame, const cv::Mat& mask);

 public:
  //
  // Getters & Setters
  //

  bool isInitialized() const
  {
    return initialized_;
  }

  void setMinFlowThresh(int min_thresh)
  {
    min_flow_thresh_= min_thresh;
  }

  void setKLTCornerThresh(double corner_thresh)
  {
    klt_corner_thresh_ = corner_thresh;
  }

  void setKLTCornerMinDist(double min_dist)
  {
    klt_corner_min_dist_ = min_dist;
  }

  void setKLTMaxCorners(int max_corners)
  {
    max_corners_ = max_corners;
  }

  void setUseFast(bool use_fast)
  {
    use_fast_ = use_fast;
  }
  void stop() { initialized_ = false; }

  AffineFlowMeasures getMostRecentFlow() const
  {
    return cur_flow_;
  }

  Descriptors getMostRecentDescriptors() const
  {
    return cur_descriptors_;
  }

  KeyPoints getMostRecentKeyPoints() const
  {
    return cur_keypoints_;
  }

  std::vector<float> getMostRecentScores() const
  {
    return cur_scores_;
  }

 public:
  cv::SURF surf_;
 protected:
  KeyPoints prev_keypoints_;
  KeyPoints cur_keypoints_;
  Descriptors prev_descriptors_;
  Descriptors cur_descriptors_;
  AffineFlowMeasures cur_flow_;
  std::vector<float> cur_scores_;
  bool initialized_;
  double ratio_threshold_;
  std::string window_name_;
  int min_flow_thresh_;
  int max_corners_;
  double klt_corner_thresh_;
  double klt_corner_min_dist_;
  bool use_fast_;
};
}
#endif // feature_tracker_h_DEFINED
