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

#include <cpl_visual_features/features/gabor_filter_bank.h>
#include <sstream>

using cv::Mat;
using std::vector;

namespace cpl_visual_features
{

//
// Constructors and Initialization
//
GaborFilterBank::GaborFilterBank(int M, int N, double U_l, double U_h, int gabor_size) :
    M_(M), N_(N), U_l_(U_l), U_h_(U_h), gabor_size_(gabor_size), calc_var_(true)
{
  generateGaborFilters();
  // Setup alpha vectors
  for(int m = 0; m < M_; ++m)
  {
    vector<vector<double> > alpha_mu_m;
    vector<vector<double> > alpha_sigma_m;

    vector<double> mu_m(N_,0.0);
    vector<double> sigma_m(N_,0.0);
    alpha_mu_.push_back(mu_m);
    alpha_sigma_.push_back(sigma_m);
    for(int n = 0; n < N_; ++n)
    {
      vector<double> mu_mn;
      vector<double> sigma_mn;
      alpha_mu_m.push_back(mu_mn);
      alpha_sigma_m.push_back(sigma_mn);
    }

    alpha_base_mu_.push_back(alpha_mu_m);
    alpha_base_sigma_.push_back(alpha_sigma_m);
  }
}

void GaborFilterBank::generateGaborFilters()
{
  double ln2 = log(2.0);
  
  // Populate the base x and y matrices for scale m
  Mat base_x(gabor_size_, gabor_size_, CV_64FC1);
  Mat base_y(gabor_size_, gabor_size_, CV_64FC1);
  for (int i = 0; i < base_x.cols; ++i)
  {
    for (int j = 0; j < base_x.rows; ++j)
    {
      base_x.at<double>(i,j) = (i - gabor_size_/2);
      base_y.at<double>(i,j) = (j - gabor_size_/2);
    }
  }

  // Iterate through the M_ scale factors
  for (int m = 0; m < M_; ++m)
  {
    vector<Mat> filters_m_c;
    vector<Mat> filters_m_w;

    // Calculate filter parameters: a and sigma_x, and sigma_y
    double a_m = pow((U_h_ / U_l_), 1.0/(m - 1.0));
    double a_m_s = pow(a_m, static_cast<double>(M_ - m));
    double u_m = U_h_/a_m_s;
    double sigma_u = ((a_m - 1.0)*u_m)/((a_m + 1.0)*sqrt(2*ln2));
    double sigma_x = 1.0/(2.0*M_PI*sigma_u);
    double z = -2.0*ln2*sigma_u*sigma_u/u_m;

    // Iterate through the N_ orientations
    for (int n = 0; n < N_; ++n)
    {
      // Calculate filter parameters theta and sigma_y
      double theta = M_PI / N_*n;
      double sigma_v = tan(M_PI/(2.0*n))*(u_m + z)/sqrt(2.0*ln2 - z*z/
                                                        (sigma_u*sigma_u));
      double sigma_y = 1.0/(2.0*M_PI*sigma_v);

      Mat x_theta = base_x *  cos(theta) + base_y * sin(theta);
      Mat y_theta = base_x * -sin(theta) + base_y * cos(theta);

      Mat gabor_mn_r(gabor_size_, gabor_size_, CV_64FC1, 1.0);
      Mat gabor_mn_i(gabor_size_, gabor_size_, CV_64FC1, 1.0);
      gabor_mn_r *= 1.0/(2.0*M_PI*sigma_x*sigma_y)*a_m_s;
      gabor_mn_i *= 1.0/(2.0*M_PI*sigma_x*sigma_y)*a_m_s;

      Mat to_exp(gabor_size_, gabor_size_, CV_64FC1);
      to_exp = (x_theta.mul(x_theta)*(1.0/sigma_x*sigma_x) +
                y_theta.mul(y_theta)*(1.0/sigma_y*sigma_y))*-0.5;

      for(int i = 0; i < gabor_size_; i++)
      {
        for(int j = 0; j < gabor_size_; j++)
        {
          gabor_mn_r.at<double>(i,j) *= exp(to_exp.at<double>(i,j))*
              cos(x_theta.at<double>(i,j)*M_PI*2.0*u_m);
          gabor_mn_i.at<double>(i,j) *= exp(to_exp.at<double>(i,j))*
              sin(x_theta.at<double>(i,j)*M_PI*2.0*u_m);
        }
      }
      gabor_mn_r*(1.0/a_m);
      gabor_mn_i*(1.0/a_m);

      // joint the two filters into a complex matrix
      Mat gabor_mn_c(gabor_size_, gabor_size_, CV_64FC2);
      vector<Mat> gabor_mns;
      gabor_mns.push_back(gabor_mn_r);
      gabor_mns.push_back(gabor_mn_i);
      merge(gabor_mns, gabor_mn_c);

      filters_m_c.push_back(gabor_mn_c);
    }
    gabor_c_.push_back(filters_m_c);
  }
}

Mat GaborFilterBank::filterImg(Mat& img, int m, int n, bool use_real)
{
  Mat bw_img(img.rows, img.cols, CV_8UC1);
  if (img.channels() == 3)
  {
    cvtColor(img, bw_img, CV_BGR2GRAY);
  }
  else
  {
    bw_img = img;
  }

  Mat convolved(bw_img.rows, bw_img.cols, bw_img.type());
  vector<Mat> components;
  split(gabor_c_[m][n], components);
  if (use_real)
    cv::filter2D(bw_img, convolved, -1, components[0]);
  else
    cv::filter2D(bw_img, convolved, -1, components[1]);
  return convolved;
}

//
// Core Functions
//
vector<double> GaborFilterBank::extractFeature(Mat& frame)
{
  vector<double> feat;
  Mat  bw_frame(frame.rows, frame.cols,  CV_8UC1);
  Mat use_frame(frame.rows, frame.cols, CV_64FC1);

  if (frame.channels() == 3)
  {
    cvtColor(frame, bw_frame, CV_BGR2GRAY);
  }
  else
  {
    bw_frame = frame;
  }
  bw_frame.convertTo(use_frame, use_frame.type());

  for(int m = 0; m < M_; ++m)
  {
    for(int n = 0; n < N_; ++n)
    {
      Mat D_r = filterImg(use_frame, m, n, true);
      Mat D_i = filterImg(use_frame, m, n, false);
      cv::imshow("Real", D_r);
      cv::imshow("Imaginary", D_i);
      cv::waitKey();

      // Calculate the mean and sd of D
      double mu_mn = mean(D_r)[0];
      Mat use_calc = D_r - mu_mn;
      double sigma_mn = std::sqrt(mean(use_calc.mul(use_calc))[0]);

      // Save to the feature and to the sample distribution
      feat.push_back(mu_mn);
      feat.push_back(sigma_mn);
      alpha_base_mu_[m][n].push_back(mu_mn);
      alpha_base_sigma_[m][n].push_back(sigma_mn);
    }
  }
  calc_var_ = true;
  return feat;
}

double GaborFilterBank::featureDist(vector<double> f1, vector<double> f2)
{
  // TODO: Add sanity check for feature lengths

  double dist = 0.0;

  if (calc_var_)
    calcSampleVar();
  
  for(int m = 0, i = 0; m < M_; ++m)
    for(int n = 0; n < N_; ++n, i += 2)
    {
      double d_mn = std::abs((f1[i] - f2[i])/alpha_mu_[m][n]) +
          std::abs((f1[i+1] - f2[i+1])/alpha_sigma_[m][n]);
      dist += d_mn;
    }

  return dist;   
}

//
// Helper functions
//

void GaborFilterBank::calcSampleVar()
{
  int I = alpha_base_mu_[0][0].size();
  for(int m = 0; m < M_; ++m)
  {
    for(int n = 0; n < N_; ++n)
    {
      double mu_mean = 0.0;
      double sigma_mean = 0.0;
      // Calculate means
      for(int i = 0; i < I; ++i)
      {
        mu_mean += alpha_base_mu_[m][n][i];
        sigma_mean += alpha_base_sigma_[m][n][i];
      }

      mu_mean /= static_cast<double>(alpha_base_mu_.size());
      sigma_mean /= static_cast<double>(alpha_base_mu_.size());

      double std_mu = 0.0;
      double std_sigma = 0.0;
      // Calculate standard deviations
      for(int i = 0; i < I; ++i)
      {
        std_mu += ((mu_mean - alpha_base_mu_[m][n][i])*
                   (mu_mean - alpha_base_mu_[m][n][i]));
        std_sigma += ((sigma_mean - alpha_base_sigma_[m][n][i])*
                      (sigma_mean - alpha_base_sigma_[m][n][i]));
      }

      std_mu = sqrt(std_mu/I);
      std_sigma = sqrt(std_sigma/I);

      alpha_mu_[m][n] = std_mu;
      alpha_sigma_[m][n] = std_sigma;
    }
  }   
}

}
