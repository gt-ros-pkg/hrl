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
#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/norms.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// STL
#include <vector>
#include <deque>
#include <string>
#include <sstream>
#include <math.h>
#include <utility>

// Boost
#include <boost/shared_ptr.hpp>

namespace tabletop_pushing
{

typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
class ProtoObject
{
 public:
  XYZPointCloud cloud;
  Eigen::Vector4f centroid;
  int id;
  bool moved;
  Eigen::Matrix4f transform;
  std::vector<int> boundary_angle_dist;
  std::vector<int> push_history;
  bool singulated;
  double icp_score;
};
typedef std::deque<ProtoObject> ProtoObjects;

class PointCloudSegmentation
{
 public:
  PointCloudSegmentation(boost::shared_ptr<tf::TransformListener> tf);

  /**
   * Function to determine the table plane in a point cloud
   *
   * @param cloud The cloud with the table as dominant plane.
   *
   * @return The centroid of the points belonging to the table plane.
   */
  Eigen::Vector4f getTablePlane(XYZPointCloud& cloud, XYZPointCloud& objs_cloud,
                                XYZPointCloud& plane_cloud,
                                bool find_concave_hull=false);

  /**
   * Function to segment independent spatial regions from a supporting plane
   *
   * @param input_cloud The point cloud to operate on.
   * @param extract_table True if the table plane should be extracted
   *
   * @return The object clusters.
   */
  ProtoObjects findTabletopObjects(XYZPointCloud& input_cloud);

  /**
   * Function to segment independent spatial regions from a supporting plane
   *
   * @param input_cloud The point cloud to operate on.
   * @param objs_cloud  The point cloud containing the object points.
   * @param extract_table True if the table plane should be extracted
   *
   * @return The object clusters.
   */
  ProtoObjects findTabletopObjects(XYZPointCloud& input_cloud,
                                   XYZPointCloud& objs_cloud);

  /**
   * Function to segment independent spatial regions from a supporting plane
   *
   * @param input_cloud The point cloud to operate on.
   * @param objs_cloud  The point cloud containing the object points.
   * @param plane_cloud  The point cloud containing the table plane points.
   * @param extract_table True if the table plane should be extracted
   *
   * @return The object clusters.
   */
  ProtoObjects findTabletopObjects(XYZPointCloud& input_cloud,
                                   XYZPointCloud& objs_cloud,
                                   XYZPointCloud& plane_cloud);

  /**
   * Function to segment point cloud regions using euclidean clustering
   *
   * @param objects_cloud The cloud of objects to cluster
   *
   * @return The independent clusters
   */
  ProtoObjects clusterProtoObjects(XYZPointCloud& objects_cloud);

  /**
   * Perform Iterated Closest Point between two proto objects.
   *
   * @param a The first object
   * @param b The second object
   *
   * @return The ICP fitness score of the match
   */
  double ICPProtoObjects(ProtoObject& a, ProtoObject& b,
                         Eigen::Matrix4f& transform);

  /**
   * Find the regions that have moved between two point clouds
   *
   * @param prev_cloud The first cloud to use in differencing
   * @param cur_cloud The second cloud to use
   *
   * @return The new set of objects that have moved in the second cloud
   */
  ProtoObjects getMovedRegions(XYZPointCloud& prev_cloud,
                               XYZPointCloud& cur_cloud, std::string suf="");

  /**
   * Match moved regions to previously extracted protoobjects
   *
   * @param objs The previously extracted objects
   * @param moved_regions The regions that have been detected as having moved
   *
   */
  void matchMovedRegions(ProtoObjects& objs, ProtoObjects& moved_regions);

  static inline double dist(pcl::PointXYZ a, pcl::PointXYZ b)
  {
    const double dx = a.x-b.x;
    const double dy = a.y-b.y;
    const double dz = a.z-b.z;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }

  static inline double dist(pcl::PointXYZ a, geometry_msgs::Point b)
  {
    const double dx = a.x-b.x;
    const double dy = a.y-b.y;
    const double dz = a.z-b.z;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }

  static inline double sqrDist(Eigen::Vector4f a, Eigen::Vector4f b)
  {
    const double dx = a[0]-b[0];
    const double dy = a[1]-b[1];
    const double dz = a[2]-b[2];
    return dx*dx+dy*dy+dz*dz;
  }

  static inline double sqrDist(pcl::PointXYZ a, pcl::PointXYZ b)
  {
    const double dx = a.x-b.x;
    const double dy = a.y-b.y;
    const double dz = a.z-b.z;
    return dx*dx+dy*dy+dz*dz;
  }

  static inline double sqrDistXY(pcl::PointXYZ a, pcl::PointXYZ b)
  {
    const double dx = a.x-b.x;
    const double dy = a.y-b.y;
    return dx*dx+dy*dy;
  }

  /**
   * Naively determine if two point clouds intersect based on distance threshold
   * between points.
   *
   * @param cloud0 First cloud for interesection test
   * @param cloud1 Second cloud for interesection test
   *
   * @return true if any points from cloud0 and cloud1 have distance less than
   * voxel_down_res_
   */
  bool cloudsIntersect(XYZPointCloud cloud0, XYZPointCloud cloud1);

  bool cloudsIntersect(XYZPointCloud cloud0, XYZPointCloud cloud1,
                       double thresh);

  bool pointIntersectsCloud(XYZPointCloud cloud, geometry_msgs::Point pt,
                            double thresh);

  float pointLineXYDist(pcl::PointXYZ p,Eigen::Vector3f vec,Eigen::Vector4f base);

  XYZPointCloud lineCloudIntersection(XYZPointCloud& cloud, Eigen::Vector3f vec,
                                      Eigen::Vector4f base);

  /**
   * Filter a point cloud to only be above the estimated table and within the
   * workspace in x, then downsample the voxels.
   *
   * @param cloud_in The cloud to filter and downsample
   *
   * @return The downsampled cloud
   */
  XYZPointCloud downsampleCloud(XYZPointCloud& cloud_in);

  /**
   * Method to project the current proto objects into an image
   *
   * @param objs The set of objects
   * @param img_in An image of correct size for the projection
   * @param target_frame The frame of the associated image
   *
   * @return Image containing the projected objects
   */
  cv::Mat projectProtoObjectsIntoImage(ProtoObjects& objs, cv::Size img_size,
                                       std::string target_frame);

  /**
   * Visualization function of proto objects projected into an image
   *
   * @param obj_img The projected objects image
   * @param objs The set of proto objects
   */
  cv::Mat displayObjectImage(cv::Mat& obj_img,
                             std::string win_name="projected objects",
                             bool use_display=true);

  void projectPointCloudIntoImage(XYZPointCloud& cloud, cv::Mat& lbl_img,
                                  std::string target_frame, unsigned int id=1);

  cv::Point projectPointIntoImage(pcl::PointXYZ cur_point_pcl,
                                  std::string point_frame,
                                  std::string target_frame);

  void projectPointCloudIntoImage(XYZPointCloud& cloud, cv::Mat& lbl_img);

  cv::Point projectPointIntoImage(geometry_msgs::PointStamped cur_point);

  cv::Point projectPointIntoImage(geometry_msgs::PointStamped cur_point,
                                  std::string target_frame);

  Eigen::Vector4f getTableCentroid()
  {
    return table_centroid_;
  }

 protected:
  boost::shared_ptr<tf::TransformListener> tf_;
  Eigen::Vector4f table_centroid_;

 public:
  std::vector<cv::Vec3f> colors_;
  double min_table_z_;
  double max_table_z_;
  double min_workspace_x_;
  double max_workspace_x_;
  double min_workspace_z_;
  double max_workspace_z_;
  double table_ransac_thresh_;
  double table_ransac_angle_thresh_;
  double cluster_tolerance_;
  double cloud_diff_thresh_;
  int min_cluster_size_;
  int max_cluster_size_;
  double voxel_down_res_;
  double cloud_intersect_thresh_;
  double hull_alpha_;
  bool use_voxel_down_;
  int num_downsamples_;
  sensor_msgs::CameraInfo cam_info_;
  std_msgs::Header cur_camera_header_;
  int moved_count_thresh_;
  int icp_max_iters_;
  double icp_transform_eps_;
  double icp_max_cor_dist_;
  double icp_ransac_thresh_;
};
};
