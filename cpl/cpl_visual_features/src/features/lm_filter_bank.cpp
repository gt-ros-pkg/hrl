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

#include <cpl_visual_features/features/lm_filter_bank.h>
#include <opencv2/ml/ml.hpp>
#include <sstream>
#include <fstream>
#include <algorithm>

using cv::Mat;
using std::vector;

namespace cpl_visual_features
{

//
// Constructors and Initialization
//
LMFilterBank::LMFilterBank(int orientations, int elongated_scales,
                           int gaussian_scales, int extract_scales,
                           int support) :
    n_orientations_(orientations), n_e_scales_(elongated_scales),
    n_g_scales_(gaussian_scales), n_extract_scales_(extract_scales),
    support_(support), n_filters_(0)
{
  generateFilters();
}

void LMFilterBank::generateFilters(void)
{
  // Get the standard deviations to use
  vector<float> kernel_sds;
  for (int i = 1; i <= std::max(n_e_scales_, n_g_scales_); ++i)
  {
    float scale_sigma = pow(sqrt(2.0),i);
    kernel_sds.push_back(scale_sigma);
  }

  // Build the base values for creating the filters
  Mat org_pts(2, support_*support_, CV_32FC1);
  int h_sup = (support_ - 1) / 2;

  for(int c = 0, v1_count = 0, v1 = -h_sup; c < org_pts.cols; ++c)
  {
    // Row 1 is -24 49 times, then -23 49 times, ...
    org_pts.at<float>(0,c) = v1;
    org_pts.at<float>(1,c) = -(c%support_) + h_sup;
    v1_count++;
    if (v1_count == support_)
    {
      v1++;
      v1_count = 0;
    }
  }

  // Generate elongated filters
  for (int s = 0; s < n_e_scales_; ++s)
  {
    for (int o = 0; o < n_orientations_; ++o)
    {
      double theta = M_PI*o/n_orientations_;
      double c_theta;
      double s_theta;
      sincos(theta, &c_theta, &s_theta);
      Mat rot_mat(2,2,CV_32FC1);
      rot_mat.at<float>(0,0) = c_theta;
      rot_mat.at<float>(0,1) = -s_theta;
      rot_mat.at<float>(1,0) = s_theta;
      rot_mat.at<float>(1,1) = c_theta;
      Mat rot_pts = rot_mat*org_pts;
      Mat rotpts_1 = rot_pts.row(0);
      Mat rotpts_2 = rot_pts.row(1);
      Mat bar = createElongatedFilter(kernel_sds[s], 0, 1,
                                      rotpts_1, rotpts_2, support_);
      Mat edge = createElongatedFilter(kernel_sds[s], 0, 2,
                                       rotpts_1, rotpts_2, support_);
      bars_.push_back(bar);
      edges_.push_back(edge);
    }
  }

  // Create low-pass gaussian filters and the log filters
  for (int s = 0; s < n_g_scales_; ++s)
  {
    int ksize = support_;
    Mat g1 = cv::getGaussianKernel(ksize, kernel_sds[s], CV_32F);
    Mat g1_s_n = g1 * g1.t();
    gaussians_.push_back(normalize(g1_s_n));

    Mat d1 = getDOGFilter(ksize,     kernel_sds[s]);
    Mat d2 = getDOGFilter(ksize, 3.0*kernel_sds[s]);

    dogs_.push_back(normalize(d1));
    dogs_.push_back(normalize(d2));
  }

  n_filters_ = bars_.size() + edges_.size() + gaussians_.size() + dogs_.size();
}

//
// Core Functions
//
vector<TextonFeature> LMFilterBank::extractRawFeature(Mat& frame,
                                                      bool use_all_pixels)
{
  Mat  bw_frame(frame.rows, frame.cols,  CV_8UC1);
  Mat base_frame(frame.rows, frame.cols, CV_32FC1);
  if (frame.channels() == 3)
  {
    cvtColor(frame, bw_frame, CV_BGR2GRAY);
  }
  else
  {
    bw_frame = frame;
  }
  bw_frame.convertTo(base_frame, base_frame.type());

  // For most cases we don't need to sample densely...
  int row_delta = 4;
  int col_delta = 4;
  if (use_all_pixels)
  {
    row_delta = 1;
    col_delta = 1;
  }

  // Get values based on the step sizes
  const int row_elems = frame.rows/row_delta;
  const int col_elems = frame.cols/col_delta;

  // Initialize an empty feature vector so that we can simply set values
  // directly instead of using push_back
  vector<TextonFeature> feats(row_elems*col_elems,
                              TextonFeature(n_filters_*n_extract_scales_, 0.0));
  vector<Mat> use_frames;
  cv::buildPyramid(base_frame, use_frames, n_extract_scales_);
  // Counter for all n_filter_*n_extract_scale_ filters
  int f_count = 0;

  for (int s = 0; s < n_extract_scales_; ++s)
  {
    ROS_DEBUG_STREAM("Extracting at scale: " << s);
    Mat use_frame = use_frames[s];
    // Convolve with the edge filters
    for(unsigned int i = 0; i < edges_.size(); ++i, ++f_count)
    {
      ROS_DEBUG_STREAM("\tExtracting edge filter " << i);
      Mat down_convolved;
      cv::filter2D(use_frame, down_convolved, -1, edges_[i]);

      Mat convolved = upSampleResponse(down_convolved, s, frame.size());

      for (int r = 0, r_set = 0; r_set < row_elems && r < convolved.rows;
           r += row_delta, ++r_set)
      {
        for (int c = 0, c_set = 0; c_set < col_elems && c < convolved.cols;
             c += col_delta, ++c_set)
        {
          feats[r_set*col_elems + c_set][f_count] = convolved.at<float>(r,c);
        }
      }
    }

    // Convolve with the bar filters
    for(unsigned int i = 0; i < bars_.size(); ++i, ++f_count)
    {
      ROS_DEBUG_STREAM("\tExtracting bar filter " << i);
      Mat down_convolved;
      cv::filter2D(use_frame, down_convolved, -1, bars_[i]);
      Mat convolved = upSampleResponse(down_convolved, s, frame.size());
      for (int r = 0, r_set = 0; r_set < row_elems && r < convolved.rows;
           r += row_delta, ++r_set)
      {
        for (int c = 0, c_set = 0; c_set < col_elems && c < convolved.cols;
             c += col_delta, ++c_set)
        {
          feats[r_set*col_elems + c_set][f_count] = convolved.at<float>(r,c);
        }
      }
    }

    // Conovle with the gaussians
    for(unsigned int i = 0; i < gaussians_.size(); ++i, ++f_count)
    {
      ROS_DEBUG_STREAM("\tExtracting gaussian filter " << i);

      Mat down_convolved;
      cv::filter2D(use_frame, down_convolved, -1, gaussians_[i]);
      Mat convolved = upSampleResponse(down_convolved, s, frame.size());

      for (int r = 0, r_set = 0; r_set < row_elems && r < convolved.rows;
           r += row_delta, ++r_set)
      {
        for (int c = 0, c_set = 0; c_set < col_elems && c < convolved.cols;
             c += col_delta, ++c_set)
        {
          feats[r_set*col_elems + c_set][f_count] = convolved.at<float>(r,c);
        }
      }
    }

    // Convolve with the difference of gaussians
    for(unsigned int i = 0; i < dogs_.size(); ++i, ++f_count)
    {
      ROS_DEBUG_STREAM("\tExtracting DoG filter " << i);
      Mat down_convolved;
      cv::filter2D(use_frame, down_convolved, -1, dogs_[i]);
      Mat convolved = upSampleResponse(down_convolved, s, frame.size());

      for (int r = 0, r_set = 0; r_set < row_elems && r < convolved.rows;
           r += row_delta, ++r_set)
      {
        for (int c = 0, c_set = 0; c_set < col_elems && c < convolved.cols;
             c += col_delta, ++c_set)
        {
          feats[r_set*col_elems + c_set][f_count] = convolved.at<float>(r,c);
        }
      }
    }
  }
  return feats;
}

vector<int> LMFilterBank::extractDescriptor(Mat& frame)
{
  vector<TextonFeature> raw = extractRawFeature(frame);
  vector<int> desc(centers_.size(), 0);
  for (unsigned int i = 0; i < raw.size(); ++i)
  {
    int lbl = quantizeFeature(raw[i]);
    desc[lbl] += 1;
  }
  return desc;
}

Mat LMFilterBank::textonQuantizeImage(Mat& frame)
{
  ROS_INFO_STREAM("Extracting raw features");

  vector<TextonFeature> feat = extractRawFeature(frame, true);
  ROS_INFO_STREAM("Quantizing pixels");

  Mat quantized(frame.rows, frame.cols, CV_8UC1);
  for (int r = 0, i = 0; r < frame.rows; ++r)
  {
    for (int c = 0; c < frame.cols; ++c, ++i)
    {
      // For each raw feature in the image find its label
      TextonFeature current = feat[i];
      int lbl = quantizeFeature(current);
      quantized.at<uchar>(r,c) = lbl;
    }
  }
  return quantized;
}

int LMFilterBank::quantizeFeature(TextonFeature feat)
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

vector<TextonFeature> LMFilterBank::clusterTextons(
    vector<vector<TextonFeature> > samples, int k, int attempts)
{
  // Convert the input samples into the correct format of a row per sample.
  int num_rows = samples.size()*samples[0].size();
  Mat row_samples(num_rows, n_filters_*n_extract_scales_, CV_32FC1);

  ROS_INFO_STREAM("Converting input data type");

  for (unsigned int i = 0, dst_row = 0; i < samples.size(); ++i)
  {
    for (unsigned int j = 0; j < samples[i].size(); ++j, ++dst_row)
    {
      for (unsigned int k = 0; k < samples[i][j].size(); ++k)
      {
        row_samples.at<float>(dst_row, k) = samples[i][j][k];
      }
    }
  }

  // Construct variables needed for running kmeans
  Mat labels;
  Mat centers(k, n_filters_*n_extract_scales_, CV_32FC1);
  float epsilon = 0.01;
  int kmeans_max_iter = 100000;

  ROS_INFO_STREAM("Performing kmeans clustering");

  double compactness = cv::kmeans(row_samples, k, labels,
                                  cv::TermCriteria(CV_TERMCRIT_EPS +
                                                   CV_TERMCRIT_ITER,
                                                   kmeans_max_iter, epsilon),
                                  attempts, cv::KMEANS_PP_CENTERS, centers);

  ROS_INFO_STREAM("Compactness is: " << compactness);
  ROS_INFO_STREAM("Converting center data type");

  vector<TextonFeature> texton_centers;
  for (int r = 0; r < centers.rows; ++r)
  {
    TextonFeature feat;
    for (int c = 0; c < centers.cols; ++c)
    {
      feat.push_back(centers.at<float>(r,c));
    }
    texton_centers.push_back(feat);
  }
  return texton_centers;
}

//
// Helper functions
//

Mat LMFilterBank::createElongatedFilter(float scale, int phase_x, int phase_y,
                                        Mat pts_1, Mat pts_2, int sup)
{
  Mat gx = getGaussianDeriv(3.0*scale, 0, pts_1, phase_x);
  Mat gy = getGaussianDeriv(scale, 0, pts_2, phase_y);
  Mat gxy = gx.mul(gy);
  gxy = gxy.reshape(0, sup);
  Mat f = normalize(gxy);
  return f;
}

Mat LMFilterBank::getGaussianDeriv(float sigma, float mu, Mat x, int ord)
{
  Mat X = x - mu;
  Mat numerator = X.mul(X);
  float variance = sigma*sigma;
  float denominator = 2.0*variance;
  Mat to_exp = (numerator*-1.0)/denominator;
  Mat g(X.size(), CV_32FC1, 1.0/(M_PI*sqrt(denominator)) );

  for(int r = 0; r < g.rows; ++r)
  {
    for(int c = 0; c < g.cols; ++c)
    {
      g.at<float>(r,c) *= std::exp(to_exp.at<float>(r,c));
    }
  }

  switch(ord)
  {
    case 1:
      g = -1.0*g.mul(x/variance);
      break;
    case 2:
      g = g.mul((numerator - variance) / (variance*variance));
      break;
    default:
      break;
  }

  return g;
}

Mat LMFilterBank::getDOGFilter(int ksize, float sigma, float alpha)
{
  Mat kernel(ksize, ksize, CV_32FC1);
  Mat g1 = cv::getGaussianKernel(ksize, sigma, CV_32F);
  Mat g2 = cv::getGaussianKernel(ksize, alpha*sigma, CV_32F);
  Mat G1 = g1*g1.t();
  Mat G2 = g2*g2.t();
  Mat G3 = G2 - G1;
  return G3;
}

Mat LMFilterBank::upSampleResponse(Mat& m_s, int s, cv::Size size0)
{
  // Upsample from scale s to scale 0
  Mat m_s_prime;
  Mat temp;
  m_s.copyTo(m_s_prime);
  m_s.copyTo(temp);

  // Calculate the correct sizes to up sample to
  vector<cv::Size> sizes;
  cv::Size current_size = size0;
  for (int i = 0; i < s; i++)
  {
    sizes.push_back(current_size);
    current_size.width /= 2;
    current_size.height /= 2;
  }

  for (int i = 0; i < s; i++)
  {
    cv::Size up_size;
    up_size = sizes.back();
    sizes.pop_back();
    cv::pyrUp(temp, m_s_prime, up_size);
    temp = m_s_prime;
  }

  return m_s_prime;
}

Mat LMFilterBank::normalize(Mat& f)
{
  Mat n(f.size(), CV_32FC1);
  n = f - mean(f)[0];
  n = n * 1.0/sum(abs(n))[0];
  return n;
}

float LMFilterBank::featureDist(TextonFeature f1, TextonFeature f2)
{
  // Add sanity check for feature lengths
  ROS_ASSERT_MSG(f1.size() == f2.size(),
                 "f1 has size: %d. f2 has size: %d",
                 (int)f1.size(), (int)f2.size());
  float dist = 0.0;

  for (unsigned int i = 0; i < f1.size(); ++i)
  {
    dist += std::abs(f1[i]-f2[i]);
  }

  return dist;
}

//
// User IO methods
//
void LMFilterBank::visualizeKernels(void)
{
  for(unsigned int i = 0; i < edges_.size(); ++i)
  {
    double min_val, max_val;
    cv::minMaxLoc(edges_[i], &min_val, &max_val);
    Mat viewable;
    edges_[i].convertTo(viewable, CV_32F, 1.0/max_val);
    std::stringstream name;
    name << "Edge" << i << std::endl;
    cv::imshow(name.str(), viewable);
  }
  for(unsigned int i = 0; i < bars_.size(); ++i)
  {
    double min_val, max_val;
    cv::minMaxLoc(bars_[i], &min_val, &max_val);
    Mat viewable;
    bars_[i].convertTo(viewable, CV_32F, 1.0/max_val);
    std::stringstream name;
    name << "Bar" << i << std::endl;
    cv::imshow(name.str(), viewable);

  }
  for(unsigned int i = 0; i < gaussians_.size(); ++i)
  {
    double min_val, max_val;
    cv::minMaxLoc(gaussians_[i], &min_val, &max_val);
    Mat viewable;
    gaussians_[i].convertTo(viewable, CV_32F, 1.0/max_val);
    std::stringstream name;
    name << "Gaussian" << i << std::endl;
    cv::imshow(name.str(), viewable);

  }
  for(unsigned int i = 0; i < dogs_.size(); ++i)
  {
    double min_val, max_val;
    cv::minMaxLoc(dogs_[i], &min_val, &max_val);
    Mat viewable;
    dogs_[i].convertTo(viewable, CV_32F, 1.0/max_val);
    std::stringstream name;
    name << "DOGs" << i << std::endl;
    cv::imshow(name.str(), viewable);
  }
  cv::waitKey();
}

void LMFilterBank::saveTextonCenters(vector<TextonFeature> centers,
                                     std::string file_name)
{
  ROS_INFO_STREAM("Saving texton centers of size " << centers.size()
                   << " to " << file_name.c_str());
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

bool LMFilterBank::readTextonCenters(std::string file_name)
{
  std::ifstream input(file_name.c_str());

  if (!input.good())
  {
    ROS_ERROR_STREAM("Failed to open file" << file_name);
    return false;
  }

  // Clear the centers currently loaded in the class
  centers_.clear();

  char line[1024];

  // Iterate through the lines of the file
  while( input.good() )
  {
    std::stringstream input_line(std::stringstream::in |
                                 std::stringstream::out);
    input.getline(line, 1024);
    input_line << line;

    TextonFeature center;

    // Push back the line elements
    for(int i = 0; i < n_filters_*n_extract_scales_ && !input.eof(); ++i)
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
  ROS_INFO_STREAM("Loaded texton codebook of size: " << centers_.size());
  return true;
}

}
