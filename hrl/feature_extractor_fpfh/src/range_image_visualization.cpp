/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 *
 */

/* \author Bastian Steder */

/* ---[ */


#include <iostream>
using namespace std;

#include "pcl/common/common_headers.h"
#include "pcl/range_image/range_image.h"
#include "pcl/io/pcd_io.h"
#include "pcl_visualization/range_image_visualizer.h"
#include "pcl_visualization/pcl_visualizer.h"

using namespace pcl;
using namespace pcl_visualization;
typedef PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;

// --------------
// -----Help-----
// --------------
void printUsage(const char* progName)
{
  cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
       << "Options:\n"
       << "-------------------------------------------\n"
       << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
       << "-c <int>     coordinate frame (default "<<(int)coordinate_frame<<")\n"
       << "-h           this help\n"
       << "\n\n";
}

// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  for (char c; (c = getopt(argc, argv, "r:c:h")) != -1; ) {
    switch (c) {
      case 'r':
      {
        angular_resolution = strtod(optarg, NULL);
        cout << "Setting angular resolution to "<<angular_resolution<<".\n";
        break;
      }
      case 'c':
      {
        coordinate_frame = (RangeImage::CoordinateFrame)strtol(optarg, NULL, 0);
        cout << "Using coordinate frame "<<(int)coordinate_frame<<".\n";
        break;
      }
      case 'h':
        printUsage(argv[0]);
        exit(0);
    }
  }
  angular_resolution = deg2rad(angular_resolution);
  
  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  // Read/Generate point cloud
  pcl::PointCloud<PointType> point_cloud;
  PointCloud<PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
  if (optind < argc)
  {
    sensor_msgs::PointCloud2 point_cloud_data;
    if (pcl::io::loadPCDFile(argv[optind], point_cloud_data) == -1)
    {
      ROS_ERROR_STREAM("Was not able to open file \""<<argv[optind]<<"\".\n");
      printUsage(argv[0]);
      exit(0);
    }
    fromROSMsg(point_cloud_data, point_cloud);
    RangeImage::extractFarRanges(point_cloud_data, far_ranges);
    if (pcl::getFieldIndex(point_cloud_data, "vp_x")>=0)
    {
      cout << "Scene point cloud has viewpoint information.\n";
      PointCloud<PointWithViewpoint> tmp_pc;  fromROSMsg(point_cloud_data, tmp_pc);
      Eigen::Vector3f average_viewpoint = RangeImage::getAverageViewPoint(tmp_pc);
      scene_sensor_pose = Eigen::Translation3f(average_viewpoint) * scene_sensor_pose;
    }
  }
  else
  {
    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.points.push_back(point);
      }
    }
    point_cloud.width = point_cloud.points.size();  point_cloud.height = 1;
  }

  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  RangeImage range_image;
  range_image.createFromPointCloud(point_cloud, angular_resolution, deg2rad(360.0f), deg2rad(180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges(far_ranges);

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  PCLVisualizer viewer("3D Viewer");
  viewer.addCoordinateSystem(1.0f);
  viewer.addPointCloud(point_cloud, "original point cloud");
  //viewer.addPointCloud(range_image, "range image");
  
  // --------------------------
  // -----Show range image-----
  // --------------------------
  RangeImageVisualizer range_image_widget("Range image");
  range_image_widget.setRangeImage(range_image);
  
  //--------------------
  // -----Main loop-----
  //--------------------
  while(!viewer.wasStopped() || range_image_widget.isShown())
  {
    ImageWidgetWX::spinOnce();  // process GUI events
    viewer.spinOnce(100);
    usleep(100000);
  }
}
