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

#include <opencv2/highgui/highgui.hpp>
#include <sstream>

#include <cpl_visual_features/saliency/center_surround.h>
#include <cpl_visual_features/features/gabor_filter_bank.h>
#include <cpl_visual_features/features/lm_filter_bank.h>
#include <ros/package.h>
#include <time.h> // for srand(time(NULL))
#include <cstdlib> // for MAX_RAND
#include <string>

using cv::Mat;
using cv::Rect;
using cv::Size;
using std::pair;
using std::vector;
using namespace cpl_visual_features;


int main(int argc, char** argv)
{
  srand(time(NULL));
  int count = 1;
  std::string path = "";
  bool flip_rgb = false;

  if (argc > 1)
    path = argv[1];

  if (argc > 2)
    count = atoi(argv[2]);

  if (argc > 3)
    flip_rgb = (atoi(argv[3]) != 0);

  std::stringstream texton_path;
  if (argc > 4)
  {
    texton_path.clear();
    texton_path << argv[4];
  }
  else
  {
    std::string package_path = ros::package::getPath( "cpl_visual_features");
    texton_path << package_path << "/cfg/textons-500.txt";
  }

  std::cout << "Building filters" << std::endl;
  LMFilterBank lfb;
  if (! lfb.readTextonCenters(texton_path.str()) ) return -1;

  vector<cv::Scalar> colors;
  for (int i = 0; i < count; i++)
  {
    std::stringstream filepath;
    if (count == 1 && path != "")
    {
      filepath << path;
    }

    else if (path != "")
    {
      filepath << path << i << ".png";
    }
    else
    {
      filepath << "/home/thermans/data/robot.jpg";
    }
    std::cout << "Image " << i << std::endl;
    Mat frame;
    frame = cv::imread(filepath.str());
    if (flip_rgb)
    {
      Mat op_frame(frame.rows, frame.cols, frame.type());
      cvtColor(frame, op_frame, CV_RGB2BGR);
      op_frame.copyTo(frame);
    }
    try
    {

      // vector<int> hist = lfb.extractDescriptor(frame);
      // for (unsigned int j = 0; j < hist.size(); ++j)
      // {
      //   std::cout << "\t[" << j << "]:" << hist[j] << std::endl;
      // }

      // Create max_val random colors
      Mat lbl_img = lfb.textonQuantizeImage(frame);

      int max_lbl = lfb.getNumCenters();
      std::cout << "Have " << max_lbl << " codewords." << std::endl;

      std::vector<int> texture_feat_vect = lfb.extractDescriptor(frame);
      int empty_bin_count = 0;
      for (unsigned int idx = 0; idx < texture_feat_vect.size(); ++idx)
      {
        // TODO: Look at sparsity / distribution of textures.
        if (texture_feat_vect[idx] <  1)
          ++empty_bin_count;
      }
      ROS_INFO_STREAM("There are " << empty_bin_count << " empty histogram bins.");

      if (colors.size() != static_cast<unsigned int>(max_lbl)+1)
      {
        for (int lbl = 0; lbl <= max_lbl; ++lbl)
        {
          cv::Scalar rand_color;
          rand_color[0] = (static_cast<float>(rand()) /
                           static_cast<float>(RAND_MAX));
          rand_color[1] = (static_cast<float>(rand()) /
                           static_cast<float>(RAND_MAX));
          rand_color[2] = (static_cast<float>(rand()) /
                           static_cast<float>(RAND_MAX));
          colors.push_back(rand_color);
        }
      }
      // Create a color image using the label colors
      Mat lbl_img_color_r(lbl_img.rows, lbl_img.cols, CV_32FC1);
      Mat lbl_img_color_g(lbl_img.rows, lbl_img.cols, CV_32FC1);
      Mat lbl_img_color_b(lbl_img.rows, lbl_img.cols, CV_32FC1);

      for (int r = 0; r < lbl_img.rows; ++r)
      {
        for (int c = 0; c < lbl_img.cols; ++c)
        {
          int color_idx = static_cast<int>(lbl_img.at<uchar>(r,c));
          // std::cout << "Label is: " << color_idx << std::endl;
          lbl_img_color_b.at<float>(r,c) = colors[color_idx][0];
          lbl_img_color_g.at<float>(r,c) = colors[color_idx][1];
          lbl_img_color_r.at<float>(r,c) = colors[color_idx][2];
        }
      }
      vector<Mat> lbl_img_colors;
      lbl_img_colors.push_back(lbl_img_color_b);
      lbl_img_colors.push_back(lbl_img_color_g);
      lbl_img_colors.push_back(lbl_img_color_r);
      Mat lbl_img_color(lbl_img.rows, lbl_img.cols, CV_32FC3);

      cv::merge(lbl_img_colors, lbl_img_color);
      // Show the original image as well as the colored one.
      cv::imshow("Image", frame);
      cv::imshow("Textures", lbl_img_color);
      cv::waitKey();
    }
    catch(cv::Exception e)
    {
      std::cerr << e.err << std::endl;
    }
  }

  // vector<TextonFeature> tf = lfb.getCenters();
  return 0;
}
