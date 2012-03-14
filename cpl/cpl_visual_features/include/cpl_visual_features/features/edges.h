/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Georgia Institute of Technology
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
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// STL
#include <vector>

namespace cpl_visual_features
{
class LinkEdges
{
 public:
  static std::vector<std::vector<cv::Point> > edgeLink(cv::Mat& edge_img_raw,
                                        unsigned int min_length=1,
                                        bool use_displays = false)
  {
    // binarize image
    cv::Mat edge_img(edge_img_raw.size(), CV_8UC1, cv::Scalar(0));
    for (int r = 0; r < edge_img.rows; ++r)
    {
      for (int c = 0; c < edge_img.cols; ++c)
      {
        if (edge_img_raw.at<float>(r,c) != 0.0)
        {
          edge_img.at<uchar>(r,c) = 1;
        }
      }
    }

    // Clean up edge image
    removeIsolatedPixels(edge_img);
    edge_img = thinEdges(edge_img);
    // NOTE: Here we change the input image to be the cleaned up edge image
    edge_img.convertTo(edge_img_raw, CV_32FC1);

    // Find locations of edge intersections
    cv::Mat ends;
    cv::Mat junctions;
    findEndsJunctions(edge_img, ends, junctions);

    // Join edge pixels
    cv::Mat edge_img_f;
    edge_img.convertTo(edge_img_f, CV_32FC1);
    std::vector<std::vector<cv::Point> > edges;
    int edge_no = 0;
    for (int r = 0; r < edge_img.rows; ++r)
    {
      for (int c = 0; c < edge_img.cols; ++c)
      {
        if (edge_img_f.at<float>(r,c) == 1)
        {
          std::vector<cv::Point> b = trackEdge(edge_img_f, r, c, edge_no++, junctions);
          // Remove short edges
          if (b.size() < min_length) continue;
          edges.push_back(b);
        }
      }
    }
    edge_img_f = -1*edge_img_f;

    if (use_displays)
    {
      ROS_DEBUG_STREAM("Found " << edges.size() << " edges ");
      cv::Mat edge_disp_img(edge_img.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
      for (unsigned int i = 0; i < edges.size(); ++i)
      {
        cv::Vec3f rand_color;
        rand_color[0] = static_cast<float>(rand())/RAND_MAX;
        rand_color[1] = static_cast<float>(rand())/RAND_MAX;
        rand_color[2] = static_cast<float>(rand())/RAND_MAX;

        for (unsigned int j = 0; j < edges[i].size(); ++j)
        {
          edge_disp_img.at<cv::Vec3f>(edges[i][j].y, edges[i][j].x) = rand_color;
        }
      }
      cv::imshow("linked edges", edge_disp_img);
    }

    return edges;
  }

 protected:
  static void removeIsolatedPixels(cv::Mat& img)
  {
    // Find single pixel locations
    cv::Mat singles(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat point_finder_filter(3, 3, CV_8UC1, cv::Scalar(1));
    cv::filter2D(img, singles, singles.depth(), point_finder_filter);

    // Remove pixels with filter score 1
    for (int r = 0; r < img.rows; ++r)
    {
      for (int c = 0; c < img.cols; ++c)
      {
        if (singles.at<uchar>(r,c) == 1)
        {
          img.at<uchar>(r,c) = 0;
        }
      }
    }
  }

  static cv::Mat thinEdges(cv::Mat img)
  {
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp(img.size(), CV_8UC1);
    cv::Mat eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
    do
    {
      cv::erode(img, eroded, element);
      cv::dilate(eroded, temp, element);
      cv::subtract(img, temp, temp);
      cv::bitwise_or(skel, temp, skel);
      eroded.copyTo(img);
    } while (!(cv::norm(img) == 0));
    removeIsolatedPixels(skel);
    return skel;
  }

  static void findEndsJunctions(cv::Mat& edge_img, cv::Mat& ends,
                                cv::Mat& junctions)
  {
    ends.create(edge_img.size(), CV_8UC1);
    junctions.create(edge_img.size(), CV_8UC1);
    for (int r = 0; r < edge_img.rows; ++r)
    {
      for (int c = 0; c < edge_img.cols; ++c)
      {
        if (edge_img.at<uchar>(r,c))
        {
          int crossings = getCrossings(edge_img, r, c);
          if (crossings >= 6)
          {
            junctions.at<uchar>(r,c) = 1;
          }
          else if (crossings == 2)
          {
            ends.at<uchar>(r,c) = 1;
          }
        }
      }
    }
  }

  static int getCrossings(cv::Mat& edge_img, const int r, const int c)
  {
    cv::Mat a(1,8,CV_8SC1, cv::Scalar(0));
    cv::Mat b(1,8,CV_8SC1, cv::Scalar(0));
    a.at<char>(0,0) = edge_img.at<uchar>(r-1,c-1);
    a.at<char>(0,1) = edge_img.at<uchar>(r-1,c);
    a.at<char>(0,2) = edge_img.at<uchar>(r-1,c+1);
    a.at<char>(0,3) = edge_img.at<uchar>(r,c+1);
    a.at<char>(0,4) = edge_img.at<uchar>(r+1,c+1);
    a.at<char>(0,5) = edge_img.at<uchar>(r+1,c);
    a.at<char>(0,6) = edge_img.at<uchar>(r+1,c-1);
    a.at<char>(0,7) = edge_img.at<uchar>(r,c-1);

    b.at<char>(0,0) = edge_img.at<uchar>(r-1,c);
    b.at<char>(0,1) = edge_img.at<uchar>(r-1,c+1);
    b.at<char>(0,2) = edge_img.at<uchar>(r,c+1);
    b.at<char>(0,3) = edge_img.at<uchar>(r+1,c+1);
    b.at<char>(0,4) = edge_img.at<uchar>(r+1,c);
    b.at<char>(0,5) = edge_img.at<uchar>(r+1,c-1);
    b.at<char>(0,6) = edge_img.at<uchar>(r,c-1);
    b.at<char>(0,7) = edge_img.at<uchar>(r-1,c-1);
    return cv::sum(cv::abs(a-b))[0];
  }

  enum PtStatus
  {
    NO_POINT,
    THERE_IS_A_POINT,
    LAST_POINT
  };

  static std::vector<cv::Point> trackEdge(cv::Mat& edge_img, int r_start, int c_start,
                            int edge_no, cv::Mat& junctions)
  {
    std::vector<cv::Point> b;
    b.push_back(cv::Point(c_start, r_start));
    edge_img.at<float>(r_start, c_start) = -edge_no;
    int r = r_start;
    int c = c_start;
    PtStatus status = nextPoint(edge_img, r, c, edge_no, junctions);

    while (status != NO_POINT)
    {
      b.push_back(cv::Point(c, r));
      edge_img.at<float>(r,c) = -edge_no;
      if (status == LAST_POINT)
      {
        status = NO_POINT;
      }
      else
      {
        status = nextPoint(edge_img, r, c, edge_no, junctions);
      }
    }

    if (isJunction(junctions,cv::Point(c_start, r_start)))
    {
      std::reverse(b.begin(), b.end());
      // TODO: Should this call in recursively and just extend b?
      status = nextPoint(edge_img, r_start, c_start, edge_no, junctions);

      while (status != NO_POINT)
      {
        b.push_back(cv::Point(c, r));
        edge_img.at<float>(r,c) = -edge_no;
        if (status == LAST_POINT)
        {
          status = NO_POINT;
        }
        else
        {
          status = nextPoint(edge_img, r, c, edge_no, junctions);
        }
      }
    }

    // check for loops and close them
    if (b.size() >= 4)
    {
      const int end = b.size() -1;
      if (abs(b[0].x - b[end].x) <= 1 && abs(b[0].y - b[end].y) <= 1)
      {
        b.push_back(b[0]);
      }
    }
    return b;
  }

  static PtStatus nextPoint(cv::Mat& edge_img, int& r_start, int& c_start,
                            int edge_no, cv::Mat& junctions)
  {
    // Check if any neighbors are junction locations with other lines
    for (int r = std::max(0, r_start-1); r <= std::min(r_start+1, edge_img.rows-1); ++r)
    {
      for (int c = std::max(0, c_start-1); c <= std::min(c_start+1, edge_img.cols-1); ++c)
      {
        if (isJunction(junctions, r, c) && edge_img.at<float>(r,c) != -edge_no)
        {
          r_start = r;
          c_start = c;
          return LAST_POINT;
        }
      }
    }

    bool check_flag = false;
    int backup_r = 0;
    int backup_c = 0;
    for (int r = std::max(0, r_start-1); r <= std::min(r_start+1, edge_img.rows-1); ++r)
    {
      for (int c = std::max(0, c_start-1); c <= std::min(c_start+1, edge_img.cols-1); ++c)
      {
        // Skip the current pixel
        if (r == r_start && c == c_start) continue;
        if (edge_img.at<float>(r,c) == 1)
        {
          if (neighborSum(edge_img, r, c, edge_no) < 2)
          {
            r_start = r;
            c_start = c;
            return THERE_IS_A_POINT;
          }
          else
          {
            check_flag = true;
            backup_r = r;
            backup_c = c;
          }
        }
      }
    }
    if (check_flag)
    {
      r_start = backup_r;
      c_start = backup_c;
      return THERE_IS_A_POINT;
    }

    // Set return values
    r_start = 0;
    c_start = 0;
    return NO_POINT;
  }

  static int neighborSum(cv::Mat& edge_img, int r_seed, int c_seed, int edge_no)
  {
    int ns = 0;
    for (int r = std::max(0, r_seed-1); r <= std::min(r_seed+1, edge_img.rows-1); ++r)
    {
      for (int c = std::max(0, c_seed-1); c <= std::min(c_seed+1, edge_img.cols-1); ++c)
      {
        if (r == r_seed && c == c_seed) continue;
        if (edge_img.at<float>(r,c) == -edge_no) ++ns;
      }
    }
    return ns;
  }


  static bool isJunction(cv::Mat& junctions, cv::Point p)
  {
    return (junctions.at<float>(p.y, p.x)==1);
  }

  static bool isJunction(cv::Mat& junctions, int r, int c)
  {
    return (junctions.at<float>(r, c)==1);
  }
};
};
