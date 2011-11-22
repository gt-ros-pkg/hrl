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

#include <cpl_visual_features/motion/feature_tracker.h>
#include <cpl_visual_features/motion/flow_types.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <ros/ros.h>

#define DISPLAY_TRACKER_OUTPUT 1
#ifdef DISPLAY_TRACKER_OUTPUT
#include <opencv2/highgui/highgui.hpp>
#endif // DISPLAY_TRACKER_OUTPUT
namespace cpl_visual_features
{

FeatureTracker::FeatureTracker(std::string name, double hessian_thresh,
                               int num_octaves, int num_layers, bool extended,
                               bool upright, bool use_fast) :
    surf_(hessian_thresh, num_octaves, num_layers, extended, upright),
    initialized_(false), ratio_threshold_(0.5), window_name_(name),
    min_flow_thresh_(0), max_corners_(500), klt_corner_thresh_(0.3),
    klt_corner_min_dist_(2), use_fast_(false)
{
  prev_keypoints_.clear();
  cur_keypoints_.clear();
  prev_descriptors_.clear();
  cur_descriptors_.clear();
}

//
// Main tracking logic functions
//

void FeatureTracker::initTracks(cv::Mat& frame)
{
  updateCurrentDescriptors(frame, cv::Mat());
  prev_keypoints_ = cur_keypoints_;
  prev_descriptors_ = cur_descriptors_;
  initialized_ = true;
}

AffineFlowMeasures FeatureTracker::updateTracksLK(cv::Mat& cur_frame,
                                                  cv::Mat& prev_frame)
{
  AffineFlowMeasures sparse_flow;
  std::vector<cv::Point2f> prev_points;
  std::vector<cv::Point2f> new_points;
  ROS_INFO_STREAM("max_corners: " << max_corners_);
  ROS_INFO_STREAM("quality_level: " << klt_corner_thresh_);
  ROS_INFO_STREAM("min_distance: " << klt_corner_min_dist_);
  cv::goodFeaturesToTrack(prev_frame, prev_points, max_corners_,
                          klt_corner_thresh_, klt_corner_min_dist_);
  ROS_INFO_STREAM("Found " << prev_points.size() << " corners.");
  std::vector<uchar> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(prev_frame, cur_frame, prev_points, new_points,
                           status, err);
  int moving_points = 0;
  for (unsigned int i = 0; i < prev_points.size(); i++)
  {
    if (! status[i]) continue;
    int dx = prev_points[i].x - new_points[i].x;
    int dy = prev_points[i].y - new_points[i].y;
    sparse_flow.push_back(AffineFlowMeasure(new_points[i].x, new_points[i].y,
                                            dx, dy));
    if (abs(sparse_flow[i].u) + abs(sparse_flow[i].v) > min_flow_thresh_)
      moving_points++;
  }
  ROS_INFO_STREAM(window_name_ << ": num moving points: " << moving_points);

#ifdef DISPLAY_TRACKER_OUTPUT
  cv::Mat display_cur_frame(cur_frame.rows, cur_frame.cols, CV_8UC3);;
  cv::cvtColor(cur_frame, display_cur_frame, CV_GRAY2BGR);
  for (unsigned int i = 0; i < sparse_flow.size(); i++)
  {
    if (abs(sparse_flow[i].u) + abs(sparse_flow[i].v) > min_flow_thresh_)
    {
      ROS_DEBUG_STREAM("Point is moving (" << sparse_flow[i].u << ", "
                       << sparse_flow[i].v << ")");
      cv::line(display_cur_frame,
               cv::Point(sparse_flow[i].x, sparse_flow[i].y),
               cv::Point(sparse_flow[i].x + sparse_flow[i].u,
                         sparse_flow[i].y + sparse_flow[i].v),
               cv::Scalar(0,0,255), 1);
    }
  }

  cv::imshow(window_name_, display_cur_frame);
#endif // DISPLAY_TRACKER_OUTPUT
  return sparse_flow;
}

AffineFlowMeasures FeatureTracker::updateTracks(const cv::Mat& frame)
{
  return updateTracks(frame, cv::Mat());
}

AffineFlowMeasures FeatureTracker::updateTracks(const cv::Mat& frame,
                                                const cv::Mat& mask)
{
  cur_keypoints_.clear();
  cur_descriptors_.clear();
  updateCurrentDescriptors(frame, mask);

  std::vector<int> matches_cur;
  std::vector<int> matches_prev;
  AffineFlowMeasures sparse_flow;
  matches_cur.clear();
  matches_prev.clear();

  // Find nearest neighbors with previous descriptors
  findMatches(cur_descriptors_, prev_descriptors_, matches_cur, matches_prev,
              cur_scores_);
  ROS_DEBUG_STREAM(window_name_ << ": num feature matches: "
                   << matches_cur.size());
  int moving_points = 0;
  for (unsigned int i = 0; i < matches_cur.size(); i++)
  {
    int dx = prev_keypoints_[matches_prev[i]].pt.x -
        cur_keypoints_[matches_cur[i]].pt.x;
    int dy = prev_keypoints_[matches_prev[i]].pt.y -
        cur_keypoints_[matches_cur[i]].pt.y;
    sparse_flow.push_back(AffineFlowMeasure(
        cur_keypoints_[matches_cur[i]].pt.x,
        cur_keypoints_[matches_cur[i]].pt.y, dx, dy));
    if (abs(sparse_flow[i].u) + abs(sparse_flow[i].v) >= min_flow_thresh_)
      moving_points++;
  }
  ROS_DEBUG_STREAM(window_name_ << ": num moving points: " << moving_points);

#ifdef DISPLAY_TRACKER_OUTPUT
  cv::Mat display_frame(frame.rows, frame.cols, CV_8UC3);;
  cv::cvtColor(frame, display_frame, CV_GRAY2BGR);
  for (unsigned int i = 0; i < matches_cur.size(); i++)
  {
    if (abs(sparse_flow[i].u) + abs(sparse_flow[i].v) >= min_flow_thresh_)
    {
      ROS_DEBUG_STREAM("Point is moving (" << sparse_flow[i].u << ", "
                       << sparse_flow[i].v << ")");
      cv::line(display_frame,
               prev_keypoints_[matches_prev[i]].pt,
               cur_keypoints_[matches_cur[i]].pt,
               cv::Scalar(0,0,255), 1);
    }
  }

  cv::imshow(window_name_, display_frame);
#endif // DISPLAY_TRACKER_OUTPUT

  prev_keypoints_ = cur_keypoints_;
  prev_descriptors_ = cur_descriptors_;
  cur_flow_ = sparse_flow;
  return sparse_flow;
}

//
// Feature Matching Functions
//

/*
 * SSD
 *
 * @short Computes the squareroot of squared differences
 * @param a First descriptor
 * @param b second descriptor
 * @return value of squareroot of squared differences
 */
double FeatureTracker::SSD(Descriptor& a, Descriptor& b)
{
  double diff = 0;

  for (unsigned int i = 0; i < a.size(); ++i) {
    float delta = a[i] - b[i];
    diff += delta*delta;
  }

  return diff;
}

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
std::pair<int, float> FeatureTracker::ratioTest(Descriptor& a,
                                                Descriptors& bList,
                                                double threshold)
{
  double best_score = 1000000;
  double second_best = 1000000;
  int best_index = -1;

  for (unsigned int b = 0; b < bList.size(); ++b) {
    double score = 0;
    score = SSD(a, bList[b]);

    if (score < best_score) {
      second_best = best_score;
      best_score = score;
      best_index = b;
    } else if (score < second_best) {
      second_best = score;
    }
    if ( best_score / second_best > threshold) {
      best_index = -1;
    }

  }

  return std::pair<int, float>(best_index, best_score);
}

/**
 * findMatches
 *
 * @param descriptors1 List of descriptors from image 1
 * @param descriptors2 List of descriptors from image 2
 * @param matches1 Indexes of matching points in image 1 (Returned)
 * @param matches2 Indexes of matching points in image 2 (Returned)
 */
void FeatureTracker::findMatches(Descriptors& descriptors1,
                                 Descriptors& descriptors2,
                                 std::vector<int>& matches1,
                                 std::vector<int>& matches2,
                                 std::vector<float>& scores)
{
  // Determine matches using the Ratio Test method from Lowe 2004
  for (unsigned int a = 0; a < descriptors1.size(); ++a) {
    const std::pair<int,float> best_match = ratioTest(descriptors1[a],
                                                      descriptors2,
                                                      ratio_threshold_);
    const int best_index = best_match.first;
    const int best_score = best_match.second;
    if (best_index != -1) {
      matches1.push_back(a);
      matches2.push_back(best_index);
      scores.push_back(best_score);
    }
  }

  // Check that the matches are unique going the other direction
  for (unsigned int x = 0; x < matches2.size();) {
    const std::pair<int,float> best_match = ratioTest(descriptors2[matches2[x]],
                                                      descriptors1,
                                                      ratio_threshold_);
    const int best_index = best_match.first;
    const int best_score = best_match.second;
    if (best_index != matches1[x]) {
      matches1.erase(matches1.begin()+x);
      matches2.erase(matches2.begin()+x);
      scores.erase(scores.begin()+x);
    } else {
      x++;
    }
  }
}

//
// Helper Functions
//

void FeatureTracker::updateCurrentDescriptors(const cv::Mat& frame,
                                              const cv::Mat& mask)
{
  std::vector<float> raw_descriptors;
  try
  {
    if (use_fast_)
    {
      cv::Mat masked_frame(frame.size(), frame.type(), cv::Scalar(0));
      frame.copyTo(masked_frame, mask);
      cv::FAST(masked_frame, cur_keypoints_, 9, true);
      // TODO: Remove keypoints outside the mask
      surf_(frame, mask, cur_keypoints_, raw_descriptors, true);
    }
    else
    {
      surf_(frame, mask, cur_keypoints_, raw_descriptors, false);
    }
    for (unsigned int i = 0; i < raw_descriptors.size(); i += 128)
    {
      Descriptor d(raw_descriptors.begin() + i,
                   raw_descriptors.begin() + i + 128);
      cur_descriptors_.push_back(d);
    }
  }
  catch(cv::Exception e)
  {
    ROS_ERROR_STREAM(e.err);
    // std::cerr << e.err << std::endl;
  }
}
}
