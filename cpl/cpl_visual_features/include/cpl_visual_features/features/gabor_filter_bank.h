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

#ifndef gabor_filter_bank_h_DEFINED
#define gabor_filter_bank_h_DEFINED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>

namespace cpl_visual_features
{
class GaborFilterBank
{
 public:
  /** 
   * Cunstroctor for FilterBank class.  Has default values based on the
   * manjunathi-pami1996 paper.
   * 
   * @param M number of scale
   * @param N number of orientations
   * @param U_l lower frequency center of interest
   * @param U_h upper frequency center of interest
   * @param gabor_size convolution filter size
   */
  GaborFilterBank(int M=4, int N = 6, double U_l = 0.05, double U_h = 0.4,
                  int gabor_size = 7);

  /** 
   * Extract a texture feature from the given image using the filter bank.
   * 
   * @param frame The image to extract the texture feature
   * 
   * @return A feature vector of length 2*M_*N_
   */
  std::vector<double> extractFeature(cv::Mat& frame);

  /** 
   * Determine the distance between two feature vectors, taking into account the
   * distribution of all feature vectors
   * 
   * @param f1 The first feature vector
   * @param f2 The second feature vector
   * 
   * @return The distance between the two vectors in feature space
   */
  double featureDist(std::vector<double> f1, std::vector<double> f2);

  /** 
   * Convolve the given image with the specified real or imaginary gabor filter
   * at scale m and orientation n
   * 
   * @param img The image with which to convolve the filter
   * @param m The scale of the filter to use (0 <= m < M_)
   * @param n The orientation of the filter to use (0 <= n < N_)
   * @param use_real true for real component filter, false for the imaginary
   * 
   * @return The convolved image showing the filter responses
   */
  cv::Mat filterImg(cv::Mat& img, int m, int n, bool use_real=true);

 protected:

  /** 
   * Generate the gabor filters to be used in processing, parameterized by class
   * members M_, N_, U_l_, U_h_, and gabor_size_
   */
  void generateGaborFilters();

  /** 
   * Method to calculate the sample variances for the extracted features.
   */
  void calcSampleVar();

  //
  // Class Members
  //
  
  int M_; // Number of scale factors
  int N_; // Number of orientations
  double U_l_; // lower center frequency of interest
  double U_h_; // upper center frequency of interest
  int gabor_size_; // Pixel width of the gabor filter
  std::vector<std::vector<cv::Mat> > gabor_c_; // Complex gabor filters

  // Vectors to store all extracted means and variances for calculation of the
  // Sample variances
  std::vector<std::vector<std::vector<double> > > alpha_base_mu_;
  std::vector<std::vector<std::vector<double> > > alpha_base_sigma_;
  // The vectors to store the smaple variances once calulcated
  std::vector<std::vector <double> > alpha_mu_;
  std::vector<std::vector <double> > alpha_sigma_;

  bool calc_var_; // Trigger for needing to recalculate the sample variances

};
}

#endif // gabor_filter_bank_h_DEFINED
