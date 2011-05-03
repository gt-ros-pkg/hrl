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

#ifndef lm_filter_bank_h_DEFINED
#define lm_filter_bank_h_DEFINED

#include <ros/common.h> // include ROS_ASSERT
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <string>

namespace cpl_visual_features
{
  // Define the texton feature structure
  typedef std::vector<float> TextonFeature;

class LMFilterBank
{
 public:
  /**
   * Cunstroctor for LMFilterBank class.  Implementing the one used in:
   * Leung and Malik. "Representing and recognizing the visual appearance of
   * materials using three-dimensional textons." IJCV, 2001.
   *
   */
  LMFilterBank(int orientations = 6, int elongated_scales = 1,
               int gaussian_scales = 1, int num_extract_scales = 2,
               int support = 7);


  /**
   * Extract a texture feature from the given image using the filter bank.
   *
   * @param frame The image to extract the texture feature
   *
   * @return A vector of length frame.rows*frame.cols of TextonFeatures for each
   * pixel
   */
  std::vector<TextonFeature> extractRawFeature(cv::Mat& frame,
                                               bool use_all_pixels = false);

  /**
   * Extracts a histogram of the texture descriptors over the given image patch
   *
   * @param frame The image patch over which to extract the quantized descriptor
   *
   * @return The quantized texture descriptor
   */
  std::vector<int> extractDescriptor(cv::Mat& frame);

  /**
   * Determine the distance between two feature vectors.
   *
   * @param f1 The first feature vector
   * @param f2 The second feature vector
   *
   * @return The distance between the two vectors in feature space
   */
  float featureDist(TextonFeature f1, TextonFeature f2);

  /**
   * Display the different kernels.
   */
  void visualizeKernels(void);

  /**
   * Method to cluster raw extracted feauters into textons
   *
   * @param samples
   * @param k The number of texton centers to find
   *
   * @return The texton centers.
   */
  std::vector<TextonFeature> clusterTextons(
      std::vector<std::vector<TextonFeature> > samples, int k, int attempts=3);

  /**
   * Save the texton centers to a file. Each row of the file corresponds to a
   * different center.
   *
   * @param centers The vector of cluster centers to write to disk.
   * @param file_name The location to save the centers.
   */
  void saveTextonCenters(std::vector<TextonFeature> centers,
                         std::string file_name);

  /**
   * Read the cluster centers into memory that have previously been saved to
   * disk using the method saveTextonCenters. These centers will be stored
   * internally in the lm_filter_bank instance
   *
   * @param file_name The location of the cluster center files
   */
  bool readTextonCenters(std::string file_name);

  /**
   * Method to take a raw filter response feature and quantize it to the correct
   * texton value.
   *
   * @param feat A raw texton feature.
   *
   * @return The label (index) of the nearest texton center.
   */
  int quantizeFeature(TextonFeature feat);

  /**
   * For a given image associate each pixel with a texture label.
   *
   * @param frame The image on which to perform the feature extraction and
   * quantization.
   *
   * @return An image containing the correct image labels;
   */
  cv::Mat textonQuantizeImage(cv::Mat& frame);

  //
  // Getters and Setters
  //
  std::vector<TextonFeature> getCenters() const
  {
    return centers_;
  }

  int getNumCenters() const
  {
    return centers_.size();
  }

 protected:
  void generateFilters(void);
  cv::Mat getGaussianDeriv(float sigma, float mu, cv::Mat x, int ord);
  cv::Mat createElongatedFilter(float scale, int phase_x, int phase_y,
                                cv::Mat pts_1, cv::Mat pts_2, int sup);
  cv::Mat upSampleResponse(cv::Mat& down_convolved, int s, cv::Size size0);
  cv::Mat normalize(cv::Mat& f);
  cv::Mat getDOGFilter(int ksize, float sigma, float alpha=3.0);

  //
  // Class Members
  //
  int n_orientations_;
  int n_e_scales_;
  int n_g_scales_;
  int n_extract_scales_;
  int support_;
  int n_filters_;
  std::vector<cv::Mat> bars_;
  std::vector<cv::Mat> edges_;
  std::vector<cv::Mat> gaussians_;
  std::vector<cv::Mat> dogs_;
  std::vector<TextonFeature> centers_;
};
}

#endif // filter_bank_h_DEFINED
