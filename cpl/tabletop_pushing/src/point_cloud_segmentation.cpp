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

// TF
#include <tf/transform_datatypes.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
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

// STL
#include <sstream>
// Local
#include <tabletop_pushing/point_cloud_segmentation.h>

// #define DISPLAY_CLOUD_DIFF 1
#define randf() static_cast<float>(rand())/RAND_MAX

typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

namespace tabletop_pushing
{

PointCloudSegmentation::PointCloudSegmentation(
    boost::shared_ptr<tf::TransformListener> tf) :
    tf_(tf)
{
  for (int i = 0; i < 200; ++i)
  {
    cv::Vec3f rand_color;
    rand_color[0] = randf();
    rand_color[1] = randf()*0.5;
    rand_color[2] = randf();
    colors_.push_back(rand_color);
  }
}

/**
 * Function to determine the table plane in a point cloud
 *
 * @param cloud The cloud with the table as dominant plane.
 *
 * @return The centroid of the points belonging to the table plane.
 */
Eigen::Vector4f PointCloudSegmentation::getTablePlane(
    XYZPointCloud& cloud, XYZPointCloud& objs_cloud, XYZPointCloud& plane_cloud,
    bool find_concave_hull)
{
  XYZPointCloud cloud_downsampled;
  if (use_voxel_down_)
  {
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(cloud.makeShared());
    downsample.setLeafSize(voxel_down_res_, voxel_down_res_, voxel_down_res_);
    downsample.filter(cloud_downsampled);
  }

  // Filter Cloud to not look for table planes on the ground
  XYZPointCloud cloud_z_filtered, cloud_filtered;
  pcl::PassThrough<pcl::PointXYZ> z_pass;
  if (use_voxel_down_)
  {
    z_pass.setInputCloud(cloud_downsampled.makeShared());
  }
  else
  {
    z_pass.setInputCloud(cloud.makeShared());
  }
  z_pass.setFilterFieldName("z");
  z_pass.setFilterLimits(min_table_z_, max_table_z_);
  z_pass.filter(cloud_z_filtered);

  // Filter to be just in the range in front of the robot
  pcl::PassThrough<pcl::PointXYZ> x_pass;
  x_pass.setInputCloud(cloud_z_filtered.makeShared());
  x_pass.setFilterFieldName("x");
  x_pass.setFilterLimits(min_workspace_x_, max_workspace_x_);
  x_pass.filter(cloud_filtered);

  // Segment the tabletop from the points using RANSAC plane fitting
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices plane_inliers;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
  plane_seg.setOptimizeCoefficients(true);
  plane_seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  plane_seg.setMethodType(pcl::SAC_RANSAC);
  plane_seg.setDistanceThreshold(table_ransac_thresh_);
  plane_seg.setInputCloud(cloud_filtered.makeShared());
  Eigen::Vector3f v(1.0,1.0,0.0);
  plane_seg.setAxis(v);
  plane_seg.setEpsAngle(table_ransac_angle_thresh_);
  plane_seg.segment(plane_inliers, coefficients);
  pcl::copyPointCloud(cloud_filtered, plane_inliers, plane_cloud);

  // Extract the outliers from the point clouds
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices plane_outliers;
  extract.setInputCloud(cloud_filtered.makeShared());
  extract.setIndices(boost::make_shared<pcl::PointIndices>(plane_inliers));
  extract.setNegative(true);
  extract.filter(objs_cloud);

  // Estimate hull from the inlier points
  if (find_concave_hull)
  {
    ROS_INFO_STREAM("finding concave hull. Plane size: " <<
                    plane_cloud.size());
    XYZPointCloud hull_cloud;
    pcl::ConcaveHull<pcl::PointXYZ> hull;
    hull.setInputCloud(plane_cloud.makeShared());
    hull.setAlpha(hull_alpha_);
    hull.reconstruct(hull_cloud);
    ROS_INFO_STREAM("hull_cloud.size() " << hull_cloud.size());
    // TODO: Return the hull_cloud
    // TODO: Figure out if stuff is inside the hull
  }

  // Extract the plane members into their own point cloud
  Eigen::Vector4f table_centroid;
  pcl::compute3DCentroid(plane_cloud, table_centroid);
  return table_centroid;
}


/**
 * Function to segment independent spatial regions from a supporting plane
 *
 * @param input_cloud The point cloud to operate on.
 * @param extract_table True if the table plane should be extracted
 *
 * @return The object clusters.
 */
ProtoObjects PointCloudSegmentation::findTabletopObjects(XYZPointCloud& input_cloud)
{
  XYZPointCloud objs_cloud;
  return findTabletopObjects(input_cloud, objs_cloud);
}

/**
 * Function to segment independent spatial regions from a supporting plane
 *
 * @param input_cloud The point cloud to operate on.
 * @param objs_cloud  The point cloud containing the object points.
 * @param extract_table True if the table plane should be extracted
 *
 * @return The object clusters.
 */
ProtoObjects PointCloudSegmentation::findTabletopObjects(XYZPointCloud& input_cloud,
                                                         XYZPointCloud& objs_cloud)
{
  XYZPointCloud table_cloud;
  return findTabletopObjects(input_cloud, objs_cloud, table_cloud);

}

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
ProtoObjects PointCloudSegmentation::findTabletopObjects(XYZPointCloud& input_cloud,
                                                         XYZPointCloud& objs_cloud,
                                                         XYZPointCloud& plane_cloud)
{
  // Get table plane
  table_centroid_ = getTablePlane(input_cloud, objs_cloud, plane_cloud,
                                  false);
  min_workspace_z_ = table_centroid_[2];

  XYZPointCloud objects_cloud_down = downsampleCloud(objs_cloud);

  // Find independent regions
  ProtoObjects objs = clusterProtoObjects(objects_cloud_down);
  return objs;
}

/**
 * Function to segment point cloud regions using euclidean clustering
 *
 * @param objects_cloud The cloud of objects to cluster
 *
 * @return The independent clusters
 */
ProtoObjects PointCloudSegmentation::clusterProtoObjects(XYZPointCloud& objects_cloud)
{
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_cluster;
  KdTreePtr clusters_tree =
      boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  pcl_cluster.setClusterTolerance(cluster_tolerance_);
  pcl_cluster.setMinClusterSize(min_cluster_size_);
  pcl_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_cluster.setSearchMethod(clusters_tree);
  pcl_cluster.setInputCloud(objects_cloud.makeShared());
  pcl_cluster.extract(clusters);
  ROS_DEBUG_STREAM("Number of clusters found matching the given constraints: "
                   << clusters.size());

  ProtoObjects objs;
  for (unsigned int i = 0; i < clusters.size(); ++i)
  {
    // Create proto objects from the point cloud
    ProtoObject po;
    po.push_history.clear();
    po.boundary_angle_dist.clear();
    pcl::copyPointCloud(objects_cloud, clusters[i], po.cloud);
    pcl::compute3DCentroid(po.cloud, po.centroid);
    po.id = i;
    po.moved = false;
    po.transform = Eigen::Matrix4f::Identity();
    po.singulated = false;
    objs.push_back(po);
  }
  return objs;
}

/**
 * Perform Iterated Closest Point between two proto objects.
 *
 * @param a The first object
 * @param b The second object
 *
 * @return The ICP fitness score of the match
 */
double PointCloudSegmentation::ICPProtoObjects(ProtoObject& a, ProtoObject& b,
                       Eigen::Matrix4f& transform)
{
  // TODO: Investigate this!
  // pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(icp_max_iters_);
  icp.setTransformationEpsilon(icp_transform_eps_);
  icp.setMaxCorrespondenceDistance(icp_max_cor_dist_);
  icp.setRANSACOutlierRejectionThreshold(icp_ransac_thresh_);
  icp.setInputCloud(boost::make_shared<XYZPointCloud>(a.cloud));
  icp.setInputTarget(boost::make_shared<XYZPointCloud>(b.cloud));
  XYZPointCloud aligned;
  icp.align(aligned);
  double score = icp.getFitnessScore();
  transform = icp.getFinalTransformation();
  return score;
}

/**
 * Find the regions that have moved between two point clouds
 *
 * @param prev_cloud The first cloud to use in differencing
 * @param cur_cloud The second cloud to use
 *
 * @return The new set of objects that have moved in the second cloud
 */
ProtoObjects PointCloudSegmentation::getMovedRegions(XYZPointCloud& prev_cloud,
                             XYZPointCloud& cur_cloud, std::string suf)
{
  // cloud_out = prev_cloud - cur_cloud
  pcl::SegmentDifferences<pcl::PointXYZ> pcl_diff;
  pcl_diff.setDistanceThreshold(cloud_diff_thresh_);
  pcl_diff.setInputCloud(prev_cloud.makeShared());
  pcl_diff.setTargetCloud(cur_cloud.makeShared());
  XYZPointCloud cloud_out;
  pcl_diff.segment(cloud_out);
  ProtoObjects moved = clusterProtoObjects(cloud_out);

#ifdef DISPLAY_CLOUD_DIFF
  cv::Size img_size(320, 240);
  cv::Mat moved_img = projectProtoObjectsIntoImage(moved, img_size,
                                                   prev_cloud.header.frame_id);
  std::stringstream cluster_title;
  cluster_title << "moved clusters" << suf;
  displayObjectImage(moved_img, cluster_title.str());
#endif // DISPLAY_CLOUD_DIFF
  return moved;
}

/**
 * Match moved regions to previously extracted protoobjects
 *
 * @param objs The previously extracted objects
 * @param moved_regions The regions that have been detected as having moved
 *
 */
void PointCloudSegmentation::matchMovedRegions(ProtoObjects& objs, ProtoObjects& moved_regions)
{
  // Determining which previous objects have moved
  for (unsigned int i = 0; i < moved_regions.size(); ++i)
  {
    for (unsigned int j = 0; j < objs.size(); ++j)
    {
      if(cloudsIntersect(objs[j].cloud, moved_regions[i].cloud))
      {
        if (!objs[j].moved)
        {
          objs[j].moved = true;
        }
      }
    }
  }
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
bool PointCloudSegmentation::cloudsIntersect(XYZPointCloud cloud0, XYZPointCloud cloud1)
{
  int moved_count = 0;
  for (unsigned int i = 0; i < cloud0.size(); ++i)
  {
    const pcl::PointXYZ pt0 = cloud0.at(i);
    for (unsigned int j = 0; j < cloud1.size(); ++j)
    {
      const pcl::PointXYZ pt1 = cloud1.at(j);
      if (dist(pt0, pt1) < cloud_intersect_thresh_)
      {
        moved_count++;
      }
      if (moved_count > moved_count_thresh_)
      {
        return true;
      }
    }
  }
  return false;
}

bool PointCloudSegmentation::cloudsIntersect(XYZPointCloud cloud0, XYZPointCloud cloud1,
                     double thresh)
{
  for (unsigned int i = 0; i < cloud0.size(); ++i)
  {
    const pcl::PointXYZ pt0 = cloud0.at(i);
    for (unsigned int j = 0; j < cloud1.size(); ++j)
    {
      const pcl::PointXYZ pt1 = cloud1.at(j);
      if (dist(pt0, pt1) < thresh) return true;
    }
  }
  return false;
}

bool PointCloudSegmentation::pointIntersectsCloud(XYZPointCloud cloud, geometry_msgs::Point pt,
                          double thresh)
{
  for (unsigned int i = 0; i < cloud.size(); ++i)
  {
    const pcl::PointXYZ pt_c = cloud.at(i);
    if (dist(pt_c, pt) < thresh) return true;
  }
  return false;
}

float PointCloudSegmentation::pointLineXYDist(pcl::PointXYZ p,Eigen::Vector3f vec,Eigen::Vector4f base)
{
  Eigen::Vector3f x0(p.x,p.y,0.0);
  Eigen::Vector3f x1(base[0],base[1],0.0);
  Eigen::Vector3f x2 = x1+vec;
  Eigen::Vector3f num = (x0 - x1);
  num = num.cross(x0 - x2);
  Eigen::Vector3f den = x2 - x1;
  float d = num.norm()/den.norm();
  return d;
}

XYZPointCloud PointCloudSegmentation::lineCloudIntersection(XYZPointCloud& cloud, Eigen::Vector3f vec,
                                    Eigen::Vector4f base)
{
  // Define parametric model of the line defined by base and vec and
  // test cloud memebers for distance from the line, if the distance is less
  // than epsilon say it intersects and add to the output set.
  pcl::PointIndices line_inliers;
  for (unsigned int i = 0; i < cloud.size(); ++i)
  {
    const pcl::PointXYZ pt = cloud.at(i);
    if (pointLineXYDist(pt, vec, base) < cloud_intersect_thresh_)
    {
      line_inliers.indices.push_back(i);
    }
  }

  // Extract the interesecting points of the line.
  XYZPointCloud line_cloud;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud.makeShared());
  extract.setIndices(boost::make_shared<pcl::PointIndices>(line_inliers));
  extract.filter(line_cloud);
  return line_cloud;
}

/**
 * Filter a point cloud to only be above the estimated table and within the
 * workspace in x, then downsample the voxels.
 *
 * @param cloud_in The cloud to filter and downsample
 *
 * @return The downsampled cloud
 */
XYZPointCloud PointCloudSegmentation::downsampleCloud(XYZPointCloud& cloud_in)
{
  XYZPointCloud cloud_z_filtered, cloud_x_filtered, cloud_down;
  pcl::PassThrough<pcl::PointXYZ> z_pass;
  z_pass.setFilterFieldName("z");
  ROS_DEBUG_STREAM("Number of points in cloud_in is: " <<
                   cloud_in.size());
  z_pass.setInputCloud(cloud_in.makeShared());
  z_pass.setFilterLimits(min_workspace_z_, max_workspace_z_);
  z_pass.filter(cloud_z_filtered);
  ROS_DEBUG_STREAM("Number of points in cloud_z_filtered is: " <<
                   cloud_z_filtered.size());

  pcl::PassThrough<pcl::PointXYZ> x_pass;
  x_pass.setInputCloud(cloud_z_filtered.makeShared());
  x_pass.setFilterFieldName("x");
  x_pass.setFilterLimits(min_workspace_x_, max_workspace_x_);
  x_pass.filter(cloud_x_filtered);

  pcl::VoxelGrid<pcl::PointXYZ> downsample_outliers;
  downsample_outliers.setInputCloud(cloud_x_filtered.makeShared());
  downsample_outliers.setLeafSize(voxel_down_res_, voxel_down_res_,
                                  voxel_down_res_);
  downsample_outliers.filter(cloud_down);
  ROS_DEBUG_STREAM("Number of points in objs_downsampled: " <<
                   cloud_down.size());
  return cloud_down;
}

/**
 * Method to project the current proto objects into an image
 *
 * @param objs The set of objects
 * @param img_in An image of correct size for the projection
 * @param target_frame The frame of the associated image
 *
 * @return Image containing the projected objects
 */
cv::Mat PointCloudSegmentation::projectProtoObjectsIntoImage(ProtoObjects& objs, cv::Size img_size,
                                     std::string target_frame)
{
  cv::Mat obj_img(img_size, CV_8UC1, cv::Scalar(0));
  for (unsigned int i = 0; i < objs.size(); ++i)
  {
    projectPointCloudIntoImage(objs[i].cloud, obj_img,
                               cur_camera_header_.frame_id, i+1);
  }

  return obj_img;
}

/**
 * Visualization function of proto objects projected into an image
 *
 * @param obj_img The projected objects image
 * @param objs The set of proto objects
 */
cv::Mat PointCloudSegmentation::displayObjectImage(cv::Mat& obj_img,
                           std::string win_name,
                           bool use_display)
{
  cv::Mat obj_disp_img(obj_img.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
  for (int r = 0; r < obj_img.rows; ++r)
  {
    for (int c = 0; c < obj_img.cols; ++c)
    {
      unsigned int id = obj_img.at<uchar>(r,c);
      if (id > 0)
      {
        obj_disp_img.at<cv::Vec3f>(r,c) = colors_[id-1];
      }
    }
  }
  if (use_display)
  {
    cv::imshow(win_name, obj_disp_img);
  }
  return obj_disp_img;
}

void PointCloudSegmentation::projectPointCloudIntoImage(XYZPointCloud& cloud, cv::Mat& lbl_img,
                                std::string target_frame, unsigned int id)
{
  for (unsigned int i = 0; i < cloud.size(); ++i)
  {
    cv::Point img_idx = projectPointIntoImage(cloud.at(i),
                                              cloud.header.frame_id,
                                              target_frame);
    lbl_img.at<uchar>(img_idx.y, img_idx.x) = id;
  }
}

cv::Point PointCloudSegmentation::projectPointIntoImage(pcl::PointXYZ cur_point_pcl,
                                std::string point_frame,
                                std::string target_frame)
{
  geometry_msgs::PointStamped cur_point;
  cur_point.header.frame_id = point_frame;
  cur_point.point.x = cur_point_pcl.x;
  cur_point.point.y = cur_point_pcl.y;
  cur_point.point.z = cur_point_pcl.z;
  return projectPointIntoImage(cur_point, target_frame);
}

void PointCloudSegmentation::projectPointCloudIntoImage(XYZPointCloud& cloud, cv::Mat& lbl_img)
{
  for (unsigned int i = 0; i < cloud.size(); ++i)
  {
    cv::Point img_idx = projectPointIntoImage(cloud.at(i),
                                              cloud.header.frame_id,
                                              cur_camera_header_.frame_id);
    lbl_img.at<uchar>(img_idx.y, img_idx.x) = 1;
  }
}

cv::Point PointCloudSegmentation::projectPointIntoImage(geometry_msgs::PointStamped cur_point)
{
  return projectPointIntoImage(cur_point, cur_camera_header_.frame_id);
}

cv::Point PointCloudSegmentation::projectPointIntoImage(geometry_msgs::PointStamped cur_point,
                                std::string target_frame)
{
  cv::Point img_loc;
  try
  {
    // Transform point into the camera frame
    geometry_msgs::PointStamped image_frame_loc_m;
    tf_->transformPoint(target_frame, cur_point, image_frame_loc_m);

    // Project point onto the image
    img_loc.x = static_cast<int>((cam_info_.K[0]*image_frame_loc_m.point.x +
                                  cam_info_.K[2]*image_frame_loc_m.point.z) /
                                 image_frame_loc_m.point.z);
    img_loc.y = static_cast<int>((cam_info_.K[4]*image_frame_loc_m.point.y +
                                  cam_info_.K[5]*image_frame_loc_m.point.z) /
                                 image_frame_loc_m.point.z);

    // Downsample poses if the image is downsampled
    for (int i = 0; i < num_downsamples_; ++i)
    {
      img_loc.x /= 2;
      img_loc.y /= 2;
    }
  }
  catch (tf::TransformException e)
  {
    ROS_ERROR_STREAM("Fucked.");
    ROS_ERROR_STREAM(e.what());
  }
  return img_loc;
}

};
