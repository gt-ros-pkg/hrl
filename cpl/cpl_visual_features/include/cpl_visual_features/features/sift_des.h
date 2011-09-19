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

#ifndef sift_des_h_DEFINED
#define sift_des_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include "abstract_feature.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>

namespace cpl_visual_features
{

typedef std::vector<float> SIFTFeature;
typedef std::vector<SIFTFeature> SIFTFeatures;

template <int x_stride, int y_stride, int n_scales, bool extract_keypoints> class SIFTDes
    : public AbstractFeature<std::vector<float> >
{
 public:
  SIFTDes() : sift_()
  {
  }

  virtual void operator()(cv::Mat& img, cv::Rect& window)
  {
    cv::Mat img_bw(img.size(), CV_8UC1);
    cv::cvtColor(img, img_bw, CV_RGB2GRAY);
    SIFTFeatures raw_feats = extractRawFeatures(img_bw);
    descriptor_.clear();
    std::vector<float> desc(centers_.size(), 0);
    for (unsigned int i = 0; i < raw_feats.size(); ++i)
    {
      int label = quantizeFeature(raw_feats[i]);
      if (label < 0 || label > (int)desc.size())
      {
        std::cerr << "Quantized label " << label << " out of allowed range!"
                  << std::endl;
      }
      desc[label] += 1.0;
    }
    descriptor_ = desc;
  }

  virtual std::vector<float> getDescriptor() const
  {
    return descriptor_;
  }

  SIFTFeatures getCenters() const
  {
    return centers_;
  }

  SIFTFeatures extractRawFeatures(cv::Mat& frame)
  {
    SIFTFeatures feats;
    for (int s = 0; s < n_scales; ++s)
    {
      cv::Mat tmp = frame;
      if (s > 0)
      {
        cv::Size down_size;
        down_size.width = frame.cols/2;
        down_size.height = frame.rows/2;
        cv::pyrDown(tmp, frame, down_size);
      }
      cv::Mat raw_descriptor;

      if (extract_keypoints)
      {
        std::vector<cv::KeyPoint> cur_keypoints;
        sift_(frame, cv::Mat(), cur_keypoints, raw_descriptor, false);
      }
      else
      {
        std::vector<cv::KeyPoint> keypoint_locs;
        for (int x = 0; x < frame.cols; x += x_stride)
        {
          for (int y = 0; y < frame.rows; y += y_stride)
          {
            keypoint_locs.push_back(cv::KeyPoint(x,y,1));
          }
        }
        sift_(frame, cv::Mat(), keypoint_locs, raw_descriptor, true);
      }

      for (int r = 0; r < raw_descriptor.rows; ++r)
      {
        SIFTFeature f;
        for (int c = 0; c < raw_descriptor.cols; ++c)
        {
          f.push_back(raw_descriptor.at<float>(r,c));
        }
        feats.push_back(f);
      }
    }
    return feats;
  }

  SIFTFeatures clusterFeatures(SIFTFeatures samples, int k, int attempts)
  {

    // Convert the input samples into the correct format of a row per sample.
    int num_rows = samples.size();
    cv::Mat row_samples(num_rows, 128, CV_32FC1);

    for (unsigned int i = 0; i < samples.size(); ++i)
    {
      for (unsigned int j = 0; j < samples[i].size(); ++j)
      {
        row_samples.at<float>(i,j) = samples[i][j];
      }
    }

    // Construct variables needed for running kmeans
    cv::Mat labels;
    cv::Mat centers(k, 128, CV_32FC1);
    float epsilon = 0.01;
    int kmeans_max_iter = 100000;

    double compactness = cv::kmeans(row_samples, k, labels,
                                    cv::TermCriteria(CV_TERMCRIT_EPS +
                                                     CV_TERMCRIT_ITER,
                                                     kmeans_max_iter, epsilon),
                                    attempts, cv::KMEANS_PP_CENTERS, centers);

    SIFTFeatures sift_centers;
    for (int r = 0; r < centers.rows; ++r)
    {
      SIFTFeature feat;
      for (int c = 0; c < centers.cols; ++c)
      {
        feat.push_back(centers.at<float>(r,c));
      }
      sift_centers.push_back(feat);
    }

    return sift_centers;
  }

  void saveClusterCenters(SIFTFeatures centers, std::string file_name)
  {
    ROS_INFO_STREAM("Saving SIFT cluster centers to " << file_name.c_str());

    std::ofstream output(file_name.c_str());
    for (unsigned int i = 0; i < centers.size(); ++i)
    {
      for (unsigned int j = 0; j < centers[i].size(); ++j)
      {
        output << centers[i][j];
        if (j < centers[i].size() - 1)
          output << " ";
      }
      output << std::endl;
    }

    output.close();
  }

  bool loadClusterCenters(std::string file_name)
  {
    std::ifstream input(file_name.c_str());
    if (!input.good())
    {
      std::cerr << "Failed to open file" << file_name << std::endl;
      return false;
    }

    // Clear the centers currently loaded in the class
    centers_.clear();

    char line[1536];

    // Iterate through the lines of the file
    while( input.good() )
    {
      std::stringstream input_line(std::stringstream::in |
                                   std::stringstream::out);
      input.getline(line, 1536); // IS this long enough????
      input_line << line;

      SIFTFeature center;

      // Push back the line elements
      for(int i = 0; i < 128 && !input.eof(); ++i)
      {
        float val;
        input_line >> val;
        center.push_back(val);
      }
      centers_.push_back(center);
    }
    input.close();

    // This was the best way to ensure all codewords are read in, but no empty
    // centers are added
    if( centers_.back().size() != centers_[0].size() )
    {
      centers_.pop_back();
    }
    return true;
  }

  int quantizeFeature(SIFTFeature feat)
  {
    // TODO: use ANN or a kd-tree or something to speed this up
    float min_val = FLT_MAX;
    int min_idx = -1;
    for (unsigned int i = 0; i < centers_.size(); ++i)
    {
      float dist = featureDist(feat, centers_[i]);
      if (dist < min_val)
      {
        min_val = dist;
        min_idx = i;
      }
    }

    return min_idx;
  }

  float featureDist(SIFTFeature f1, SIFTFeature f2)
  {
    // Add sanity check for feature lengths
    // ROS_ASSERT_MSG(f1.size() == f2.size(),
    //                "f1 has size: %d. f2 has size: %d", f1.size(), f2.size());
    float dist = 0.0;

    for (unsigned int i = 0; i < f1.size(); ++i)
    {
      dist += std::abs(f1[i]-f2[i]);
    }

    return dist;
  }

 public:
  cv::SIFT sift_;

 protected:
  std::vector<float> descriptor_;
  SIFTFeatures centers_;
};
}
#endif // sift_des_h_DEFINED
