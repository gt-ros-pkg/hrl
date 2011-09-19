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

#include <cpl_visual_features/sliding_window.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <sstream>

#include <cpl_visual_features/saliency/center_surround.h>
#include <cpl_visual_features/features/normalized_sum.h>
#include <cpl_visual_features/features/color_histogram.h>
#include <cpl_visual_features/features/lab_color_histogram.h>
#include <cpl_visual_features/features/color_cells.h>
#include <cpl_visual_features/features/canny_edges.h>
#include <cpl_visual_features/features/my_hog.h>
#include <cpl_visual_features/features/attribute_learning_base_feature.h>
#include <cpl_visual_features/features/sift_des.h>

// #define RUN_SALIENCY 1
// #define USE_ATT_WINDOW 1
// #define USE_LAB_WINDOW 1

using cv::Mat;
using cv::Rect;
using cv::Size;
using std::pair;
using std::vector;
using namespace cpl_visual_features;

typedef ColorHistogram<16> ColorHist16;
typedef MyHOG<64, 64, 8, 8, 8, 8, 4, 4, 9> SWDHog;
typedef LabColorHistogram<24,16,16> LabColorHist16;
typedef SIFTDes<4, 4, 3, true> SIFTDesA;

void cannyWindow(Mat& frame, vector<pair<int,int> >& windows)
{
  SlidingWindowDetector<CannyEdges, CannyEdges::Descriptor> swd;
  swd.scanImage(frame, windows);
}

void hogWindow(Mat& frame, vector<pair<int,int> >& windows)
{
  SlidingWindowDetector< SWDHog, SWDHog::Descriptor> swd;
  swd.scanImage(frame, windows);
  // SWDHog mine;
  // cv::Rect r;
  // mine(frame, r);
}


void siftWindow(Mat& frame, vector<pair<int,int> >& windows)
{
  SlidingWindowDetector< SIFTDesA, SIFTDesA::Descriptor> swd;
  swd.scanImage(frame, windows);
}

LabColorHist16::Descriptor labColorHistWindow(Mat& frame, pair<int,int> window)
{
  SlidingWindowDetector<LabColorHist16, LabColorHist16::Descriptor> swd;
  swd.scanImage(frame, window, true);
  return swd.descriptors_[swd.descriptors_.size()-1];
}

void colorCellWindow(Mat& frame, vector<pair<int,int> >& windows)
{
  SlidingWindowDetector<ColorCell, ColorCell::Descriptor> swd;
  swd.scanImage(frame, windows[5]);
  vector<float> my_desc = swd.feature_.getDescriptor();
}

int main(int argc, char** argv)
{
  int count = 1;
  std::string path = "";
  bool use_gradient = false;
  bool flip_rgb = false;

  if (argc > 1)
    path = argv[1];

  if (argc > 2)
    count = atoi(argv[2]);

  if (argc > 3)
    use_gradient = (atoi(argv[3]) != 0);

  if (argc > 4)
    flip_rgb = (atoi(argv[4]) != 0);;

  vector<pair<int,int> > windows;
  // windows.push_back(pair<int,int>( 4,  4));
  // windows.push_back(pair<int,int>( 8,  8));
  // windows.push_back(pair<int,int>( 8,  16));
  // windows.push_back(pair<int,int>( 16,  8));
  // windows.push_back(pair<int,int>( 8,  32));
  // windows.push_back(pair<int,int>( 32,  8));
  // windows.push_back(pair<int,int>( 16,  16));
  // windows.push_back(pair<int,int>( 16,  32));
  // windows.push_back(pair<int,int>( 32,  16));
  // windows.push_back(pair<int,int>( 32,  32));
  // windows.push_back(pair<int,int>( 64,  32));
  // windows.push_back(pair<int,int>( 32,  64));
  // windows.push_back(pair<int,int>( 64,  64));
  windows.push_back(pair<int,int>( 32, 128));
  windows.push_back(pair<int,int>(128,  32));
  windows.push_back(pair<int,int>( 64, 128));
  windows.push_back(pair<int,int>(128,  64));
  windows.push_back(pair<int,int>( 64, 256));
  windows.push_back(pair<int,int>(256,  64));
  windows.push_back(pair<int,int>(128, 128));
  windows.push_back(pair<int,int>(128, 256));

#ifdef RUN_SALIENCY
  SlidingWindowDetector<NormalizedSum, NormalizedSum::Descriptor> swd;
  CenterSurroundMapper csm(1, 3, 2, 3);
#endif // RUN_SALIENCY

#ifdef USE_ATT_WINDOW
  SlidingWindowDetector<AttributeLearningBaseFeature,
      AttributeLearningBaseFeature::Descriptor> attribute_wd;
  std::stringstream texton_path;
  std::string package_path = ros::package::getPath( "cpl_visual_features");
  texton_path << package_path << "/cfg/textons-500.txt";
  attribute_wd.feature_.setTextonFile(texton_path.str());
#endif // USE_ATT_WINDOW

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
    ROS_INFO_STREAM("Image " << i);
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


      siftWindow(frame, windows);
      
#ifdef RUN_SALIENCY
      // Get the saliency map
      Mat saliency_img = csm(frame, use_gradient);
      cv::imshow("saliency map scaled", saliency_img);

      // Find the most salient region
      swd.feature_.resetMax();
      swd.scanImage(saliency_img, windows);

      // Report what this region is
      cv::Rect max_loc = swd.feature_.getMaxLoc();
      ROS_INFO_STREAM("max_window_loc: ("
                      << max_loc.x << ", " << max_loc.y << ", "
                      << max_loc.height << ", " << max_loc.width << ")");

      // Find the most salient pixel and report it
      swd.feature_.resetMax();
      swd.scanImage(saliency_img, pair<int,int>(1,1));
      cv::Rect max_point = swd.feature_.getMaxLoc();
      ROS_INFO_STREAM("max_point at: (" << max_point.x << ", " << max_point.y
                      << ")");

      // Display stuff
      Mat disp_img(frame.rows, frame.cols, frame.type());
      frame.copyTo(disp_img);

      int img_scale = frame.cols / saliency_img.cols;
      cv::rectangle(disp_img, max_loc.tl()*img_scale, max_loc.br()*img_scale,
                    CV_RGB(255,0,0));
      cv::rectangle(disp_img, max_point.tl()*img_scale,
                    max_point.br()*img_scale,
                    CV_RGB(0,255,0));

      cv::imshow("Most salient region", disp_img);
      if (i == count - 1)
        cv::waitKey();
      else
        cv::waitKey(30);
#endif // RUN_SALIENCY
#ifdef USE_ATT_WINDOW
      attribute_wd.scanImage(frame, pair<int,int>(frame.cols - 1,
                                                  frame.rows - 1), true);
      vector<float> ab_desc = attribute_wd.descriptors_[0];
      attribute_wd.descriptors_.clear();
      ROS_INFO_STREAM("Total feature vector size is: " << ab_desc.size());
#endif // USE_ATT_WINDOW

#ifdef USE_LAB_WINDOW
      LabColorHist16::Descriptor des = labColorHistWindow(
          frame, pair<int,int>(frame.cols - 1, frame.rows - 1));

      int zero_counts = 0;
      for (unsigned int idx = 0; idx < des.size(); ++idx)
      {
        if ( des[idx] == 0)
          ++zero_counts;
      }

      ROS_INFO_STREAM("Descriptor has " << zero_counts
                      << " empty bins from a total of "
                      << des.size() << ".");
#endif // USE_LAB_WINDOW
    }
    catch(cv::Exception e)
    {
      ROS_ERROR_STREAM(e.err);
    }
  }
  return 0;
}
