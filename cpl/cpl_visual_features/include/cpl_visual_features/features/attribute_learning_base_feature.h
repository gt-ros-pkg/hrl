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

#ifndef attribute_learning_base_feature_h_DEFINED
#define attribute_learning_base_feature_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "abstract_feature.h"
#include "lab_color_histogram.h"
#include "lm_filter_bank.h"
#include "canny_edges.h"
#include "my_hog.h"
#include "sift_des.h"
#include <string>
#include <vector>

//#define USE_CANNY_DESC 1
#define USE_LAB_DESC 1
#define USE_TEXT_DESC 1
//#define USE_HOG_DESC 1
#define USE_SIFT_DESC 1
//#define NORMALIZE_VALUES 1

namespace cpl_visual_features
{
class AttributeLearningBaseFeature : public AbstractFeature<std::vector<float> >
{
 public:

  // NOTE: We based these value as best we could off of the Dalal CVPR 05 Paper.
  typedef MyHOG<64, 64, 16, 16, 8, 8, 8, 8, 9> AttributeHOG;
  typedef LabColorHistogram<12, 6, 6> AttributeLabHist;
  typedef SIFTDes<8,8,4, false> SIFTDescriptor;

  AttributeLearningBaseFeature() { }

  virtual void operator()(cv::Mat& patch, cv::Rect& window)
  {
    cv::Mat patch_bw(patch.rows, patch.cols,  CV_8UC1);
    if (patch.channels() == 3)
    {
      cv::cvtColor(patch, patch_bw, CV_BGR2GRAY);
    }
    else
    {
      patch_bw = patch;
    }

    //
    // Extract base features
    //

#ifdef USE_CANNY_DESC
    ROS_DEBUG_STREAM("Extracting canny descriptor");
    std::vector<int> canny_desc = canny_.extractDescriptor(patch, window);
#endif // USE_CANNY_DESC

#ifdef USE_LAB_DESC
    ROS_DEBUG_STREAM("Extracting color descriptor");
    std::vector<float> color_desc = col_hist_.extractDescriptor(patch,
                                                                window);
#endif // USE_LAB_DESC

#ifdef USE_TEXT_DESC
    ROS_DEBUG_STREAM("Extracting texture descriptor");
    std::vector<int> texture_desc = lmfb_.extractDescriptor(patch_bw);
#endif // USE_TEXT_DESC

#ifdef USE_HOG_DESC
    ROS_DEBUG_STREAM("Extracting HOG descriptor");
    std::vector<float> hog_desc = hog_.extractDescriptor(patch, window);
#endif // USE_HOG_DESC

#ifdef USE_SIFT_DESC
    ROS_DEBUG_STREAM("Extracting SIFT descriptor");
    std::vector<float> sift_desc = sift_.extractDescriptor(patch, window);
    ROS_DEBUG_STREAM("Done extracting SIFT descriptor");
#endif // USE_SIFT_DESC

    //
    // Concatenate descriptors into one vector
    //
    std::vector<float> patch_desc;

#ifdef USE_CANNY_DESC
    ROS_DEBUG_STREAM("Canny descriptor: " << canny_desc.size());
    float canny_sum = sum<int>(canny_desc);
    for (unsigned int i = 0; i < canny_desc.size(); ++i)
    {
      patch_desc.push_back(canny_desc[i]);
      // ROS_DEBUG_STREAM(i << " : " << canny_desc[i]/canny_sum);
    }
#endif // USE_CANNY_DESC

#ifdef USE_LAB_DESC
    ROS_DEBUG_STREAM("Color descriptor: " << color_desc.size());
#ifdef NORMALIZE_VALUES
    float color_sum = sum<float>(color_desc);
#endif  // NORMALIZE_VALUES
    for (unsigned int i = 0; i < color_desc.size(); ++i)
    {
#ifdef NORMALIZE_VALUES
      patch_desc.push_back(color_desc[i]/color_sum);
#else
      patch_desc.push_back(color_desc[i]);
#endif

      // ROS_DEBUG_STREAM(i << " : " << color_desc[i]/color_sum);
    }
#endif // USE_LAB_DESC

#ifdef USE_TEXT_DESC
    ROS_DEBUG_STREAM("Texture descriptor: " << texture_desc.size());
#ifdef NORMALIZE_VALUES
    float texture_sum = sum<int>(texture_desc);
#endif  // NORMALIZE_VALUES
    for (unsigned int i = 0; i < texture_desc.size(); ++i)
    {
#ifdef NORMALIZE_VALUES
      patch_desc.push_back(texture_desc[i]/texture_sum);
#else
      patch_desc.push_back(texture_desc[i]);
#endif // NORMALIZE_VALUES


      // ROS_DEBUG_STREAM(i << " : " << texture_desc[i]/texture_sum);
    }
#endif //USE_TEXT_DESC

#ifdef USE_HOG_DESC
    ROS_DEBUG_STREAM("HOG descriptor: " << hog_desc.size());
    for (unsigned int i = 0; i < hog_desc.size(); ++i)
    {
      patch_desc.push_back(hog_desc[i]);
    }
#endif // USE_HOG_DESC

#ifdef USE_SIFT_DESC
    ROS_DEBUG_STREAM("SIFT descriptor: " << sift_desc.size());
    for (unsigned int i = 0; i < sift_desc.size(); ++i)
    {
      patch_desc.push_back(sift_desc[i]);
    }
#endif // USE_HOG_DESC

    ROS_DEBUG_STREAM("Total descriptor: " << patch_desc.size());
    descriptor_ = patch_desc;
  }

  virtual std::vector<float> getDescriptor() const
  {
    return descriptor_;
  }

  virtual void setTextonFile(std::string texton_file)
  {
    lmfb_.readTextonCenters(texton_file);
  }

  virtual void setSIFTFile(std::string sift_file)
  {
    if ( sift_.loadClusterCenters(sift_file) )
    {
      SIFTFeatures centers = sift_.getCenters();
      ROS_INFO_STREAM("Loaded " << centers.size() << " centers.");
    }
  }

 protected:
  std::vector<float> descriptor_;
  AttributeLabHist col_hist_;
  LMFilterBank lmfb_;
  CannyEdges canny_;
  AttributeHOG hog_;
  SIFTDescriptor sift_;

  float normalize(int val_raw, float min_val = 0.0, float max_val = 255.0)
  {
    // Normalize to the range 0 to 1
    return (val_raw - min_val)*2.0/(max_val - min_val);
  }

};
}
#endif // attribute_learning_base_feature_h_DEFINED
