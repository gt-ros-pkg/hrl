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

#include <cpl_visual_features/features/lm_filter_bank.h>

using cv::Mat;
using std::vector;
using namespace cpl_visual_features;

void usage(std::string exec_name)
{
  ROS_ERROR_STREAM("usage: "
                   << exec_name
                   << " image_path"
                   << " image_count"
                   << " image_type"
                   << " save_path"
                   << " [num_centers] [attempts]");
}

int main(int argc, char** argv)
{
  // Parameters to read from the command line
  int img_count = 1;
  int num_centers = 20;
  int attempts = 1;
  std::string img_path = "";
  std::string img_sfx = "";
  std::string centers_path = "";

  static const int min_args = 5;
  static const int max_args = 7;
  if (argc < min_args || argc > max_args)
    usage(argv[0]);

  img_path = argv[1];
  img_count = atoi(argv[2]);
  img_sfx = argv[3];
  centers_path = argv[4];
  if (argc > 5)
    num_centers = atoi(argv[5]);
  if (argc > 6)
    attempts = atoi(argv[6]);

  LMFilterBank lfb;
  // lfb.visualizeKernels();
  vector<vector<TextonFeature> > training_features;

  for (int i = 0; i < img_count; i++)
  {
    std::stringstream filepath;
    filepath << img_path << i << "." << img_sfx;

    ROS_INFO_STREAM("Extracting from image " << i );

    Mat frame;
    frame = cv::imread(filepath.str());
    try
    {
      training_features.push_back(lfb.extractRawFeature(frame));
    }
    catch(cv::Exception e)
    {
      ROS_ERROR_STREAM(e.err);
    }
  }

  vector<TextonFeature> tf = lfb.clusterTextons(training_features, num_centers,
                                                attempts);
  lfb.saveTextonCenters(tf, centers_path);
  return 0;
}
