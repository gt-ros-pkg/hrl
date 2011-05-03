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

#include <cpl_visual_features/features/sift_des.h>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <ros/ros.h>
#include <opencv2/ml/ml.hpp>

using namespace cpl_visual_features;
typedef SIFTDes<8,8,4, false> SIFTDescriptor;

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
  {
    usage(argv[0]);
    return -1;
  }

  img_path = argv[1];
  img_count = atoi(argv[2]);
  img_sfx = argv[3];
  centers_path = argv[4];
  if (argc > 5)
    num_centers = atoi(argv[5]);
  if (argc > 6)
    attempts = atoi(argv[6]);
  SIFTDescriptor sift;
  SIFTFeatures training_features;

  for (int i = 0; i < img_count; i++)
  {
    std::stringstream filepath;
    filepath << img_path << i << "." << img_sfx;

    ROS_INFO_STREAM("Extracting from image " << filepath.str() );

    cv::Mat frame;
    frame = cv::imread(filepath.str());
    try
    {
      cv::Mat frame_bw(frame.size(), CV_8UC1);
      cv::cvtColor(frame, frame_bw, CV_BGR2GRAY);
      SIFTFeatures new_feats = sift.extractRawFeatures(frame_bw);
      for (unsigned int j = 0; j < new_feats.size(); ++j)
      {
        training_features.push_back(new_feats[j]);
      }
    }
    catch(cv::Exception e)
    {
      ROS_ERROR_STREAM(e.err);
    }
  }
  ROS_INFO_STREAM("Have " << training_features.size() << " total features.");
  SIFTFeatures centers = sift.clusterFeatures(training_features,
                                              num_centers, attempts);
  sift.saveClusterCenters(centers, centers_path);
  sift.loadClusterCenters(centers_path);
  SIFTFeatures read_centers = sift.getCenters();
  if (centers.size() != read_centers.size())
  {
    ROS_WARN_STREAM("Read and written codebooks have different sizes: "
                    << " read: " << read_centers.size()
                    << " written: " << centers.size());
    return -1;
  }

  for (unsigned int i = 0; i < centers.size(); ++i)
  {
    SIFTFeature a = centers[i];
    SIFTFeature b = read_centers[i];
    if (a.size() != b.size())
    {
      ROS_WARN_STREAM("Read cluster center " << i << " has length "
                      << b.size());
      ROS_WARN_STREAM("Written cluster center " << i << " has length "
                      << a.size());
      continue;
    }
    for (unsigned int j = 0; j < a.size(); ++j)
    {
      // Compare feature elements
      if (a[j] != b[j])
      {
        //ROS_WARN_STREAM("(" << i << ", " << j << "): " << a[j] - b[j]);
      }
    }
  }

  return 0;
}
