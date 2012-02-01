/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Georgia Institute of Technology
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
// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>

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
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/registration/icp.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost
// TODO: Use these instead of passing pointers about
#include <boost/shared_ptr.hpp>

// cpl_visual_features
#include <cpl_visual_features/motion/flow_types.h>
#include <cpl_visual_features/motion/dense_lk.h>
#include <cpl_visual_features/motion/feature_tracker.h>

// tabletop_pushing
#include <tabletop_pushing/PushPose.h>
#include <tabletop_pushing/LocateTable.h>

// STL
#include <vector>
#include <deque>
#include <queue>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <utility>
#include <stdexcept>
#include <float.h>
#include <math.h>
#include <time.h> // for srand(time(NULL))
#include <cstdlib> // for MAX_RAND

// Debugging IFDEFS
#define DISPLAY_INPUT_COLOR 1
#define DISPLAY_INPUT_DEPTH 1
// #define DISPLAY_WORKSPACE_MASK 1
// #define DISPLAY_PLANE_ESTIMATE 1
// #define DISPLAY_TABLE_DISTANCES 1
#define DISPLAY_OBJECT_BOUNDARIES 1
#define DISPLAY_PROJECTED_OBJECTS 1
#define DISPLAY_PREV_PROJECTED_OBJECTS 1
#define DISPLAY_OBJECT_SPLITS 1
// #define DISPLAY_LINKED_EDGES 1
// #define DISPLAY_CHOSEN_BOUNDARY 1
#define DISPLAY_CLOUD_DIFF 1
#define DISPLAY_3D_BOUNDARIES 1
#define DISPLAY_WAIT 1

#define randf() static_cast<float>(rand())/RAND_MAX

using tabletop_pushing::PushPose;
using tabletop_pushing::LocateTable;
using geometry_msgs::PoseStamped;
using geometry_msgs::PointStamped;
using geometry_msgs::Pose2D;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2> MySyncPolicy;
typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;
typedef PushPose::Response PushVector;
using cpl_visual_features::AffineFlowMeasure;
using cpl_visual_features::AffineFlowMeasures;
using cpl_visual_features::FeatureTracker;
using cpl_visual_features::Descriptor;

class Boundary : public std::vector<cv::Point>
{
 public:
  double external_prob;
  double internal_prob;
  std::vector<pcl::PointXYZ> points3D;
  unsigned int object_id;
};

class ProtoTabletopObject
{
 public:
  XYZPointCloud cloud;
  Eigen::Vector4f centroid;
  Eigen::Vector4f delta_c;
  Eigen::Vector4f table_centroid;
  std::vector<Boundary> boundaries;
  int id;
  bool moved;
  int push_count;
};

typedef std::deque<ProtoTabletopObject> ProtoObjects;

class PointCloudSegmentation
{
 public:
  PointCloudSegmentation(FeatureTracker* ft, tf::TransformListener * tf) :
      ft_(ft), tf_(tf)
  {
    for (int i = 0; i < 200; ++i)
    {
      cv::Vec3f rand_color;
      rand_color[0] = randf();
      rand_color[1] = randf();
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
  Eigen::Vector4f getTablePlane(XYZPointCloud& cloud, XYZPointCloud& objs_cloud,
                                XYZPointCloud& plane_cloud,
                                bool find_concave_hull=false)
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
    plane_seg.segment(plane_inliers, coefficients);
    Eigen::Vector3f v(1.0,1.0,0.0);
    plane_seg.setAxis(v);
    plane_seg.setEpsAngle(table_ransac_angle_thresh_);
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
  ProtoObjects findTabletopObjects(XYZPointCloud& input_cloud,
                                   bool publish_cloud=true)
  {
    XYZPointCloud objs_cloud;
    return findTabletopObjects(input_cloud, objs_cloud, publish_cloud);
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
  ProtoObjects findTabletopObjects(XYZPointCloud& input_cloud,
                                   XYZPointCloud& objs_cloud,
                                   bool publish_cloud=false)
  {
    XYZPointCloud table_cloud;
    return findTabletopObjects(input_cloud, objs_cloud, table_cloud,
                               publish_cloud);

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
  ProtoObjects findTabletopObjects(XYZPointCloud& input_cloud,
                                   XYZPointCloud& objs_cloud,
                                   XYZPointCloud& plane_cloud,
                                   bool publish_cloud=false)
  {
    // Get table plane
    table_centroid_ = getTablePlane(input_cloud, objs_cloud, plane_cloud,
                                    false);
    min_workspace_z_ = table_centroid_[2];

    XYZPointCloud objects_cloud_down = downsampleCloud(objs_cloud);

    // Find independent regions
    ProtoObjects objs = clusterProtoObjects(objects_cloud_down, publish_cloud);
    return objs;
  }

  /**
   * Function to segment point cloud regions using euclidean clustering
   *
   * @param objects_cloud The cloud of objects to cluster
   * @param pub_cloud True if the resulting segmentation should be published
   *
   * @return The independent clusters
   */
  ProtoObjects clusterProtoObjects(XYZPointCloud& objects_cloud,
                                   bool pub_cloud = false)
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

    if (pub_cloud)
    {
      pcl::PointCloud<pcl::PointXYZI> label_cloud;
      pcl::copyPointCloud(objects_cloud, label_cloud);
      for (unsigned int i = 0; i < clusters.size(); ++i)
      {
        for (unsigned int j = 0; j < clusters[i].indices.size(); ++j)
        {
          // NOTE: Intensity 0 is the table; so use 1-based indexing
          label_cloud.at(clusters[i].indices[j]).intensity = (i+1);
        }
      }
      sensor_msgs::PointCloud2 label_cloud_msg;
      pcl::toROSMsg(label_cloud, label_cloud_msg);
      pcl_obj_seg_pub_.publish(label_cloud_msg);
    }

    ProtoObjects objs;
    for (unsigned int i = 0; i < clusters.size(); ++i)
    {
      // Create proto objects from the point cloud
      ProtoTabletopObject po;
      pcl::copyPointCloud(objects_cloud, clusters[i], po.cloud);
      pcl::compute3DCentroid(po.cloud, po.centroid);
      po.id = i;
      po.table_centroid = table_centroid_;
      po.moved = false;
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
  double ICPProtoObjects(ProtoTabletopObject& a, ProtoTabletopObject& b)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(boost::make_shared<XYZPointCloud>(a.cloud));
    icp.setInputTarget(boost::make_shared<XYZPointCloud>(b.cloud));
    XYZPointCloud aligned;
    icp.align(aligned);
    double score = icp.getFitnessScore();
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
  ProtoObjects getMovedRegions(XYZPointCloud& prev_cloud,
                               XYZPointCloud& cur_cloud, std::string suf="",
                               bool pub_cloud=false)
  {
    // cloud_out = prev_cloud - cur_cloud
    pcl::SegmentDifferences<pcl::PointXYZ> pcl_diff;
    pcl_diff.setDistanceThreshold(cloud_diff_thresh_);
    pcl_diff.setInputCloud(prev_cloud.makeShared());
    pcl_diff.setTargetCloud(cur_cloud.makeShared());
    XYZPointCloud cloud_out;
    pcl_diff.segment(cloud_out);
    ProtoObjects moved = clusterProtoObjects(cloud_out, pub_cloud);

#ifdef DISPLAY_CLOUD_DIFF
    cv::Size img_size(320, 240);
    cv::Mat coord_img(img_size, CV_32FC3, cv::Scalar(0,0,0));
    // cv::Mat obj_img(img_size, CV_8UC1, cv::Scalar(0));
    // projectPointCloudIntoImage(cloud_out, obj_img, cur_camera_header_.frame_id,
    //                            coord_img, 128);
    // std::stringstream diff_title;
    // diff_title << "cloud_diff" << suf;
    // cv::imshow(diff_title.str(), obj_img);
    cv::Mat moved_img = projectProtoObjectsIntoImage(moved, img_size,
                                                     prev_cloud.header.frame_id,
                                                     coord_img);
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
  void matchMovedRegions(ProtoObjects& objs, ProtoObjects& moved_regions)
  {
    // Determining which previous objects have moved
    int moved_count = 0;
    for (unsigned int i = 0; i < moved_regions.size(); ++i)
    {
      for (unsigned int j = 0; j < objs.size(); ++j)
      {
        if(cloudsIntersect(objs[j].cloud, moved_regions[i].cloud))
        {
          if (!objs[j].moved) ++moved_count;
          objs[j].moved = true;
        }
      }
    }
    ROS_DEBUG_STREAM("Num moved objects: " << moved_count);
  }

  double dist(pcl::PointXYZ a, pcl::PointXYZ b)
  {
    double dx = a.x-b.x;
    double dy = a.y-b.y;
    double dz = a.z-b.z;
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  }

  double sqrDist(Eigen::Vector4f a, Eigen::Vector4f b)
  {
    double dx = a[0]-b[0];
    double dy = a[1]-b[1];
    double dz = a[2]-b[2];
    return dx*dx+dy*dy+dz*dz;
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
  bool cloudsIntersect(XYZPointCloud cloud0, XYZPointCloud cloud1)
  {
    for (unsigned int i = 0; i < cloud0.size(); ++i)
    {
      pcl::PointXYZ pt0 = cloud0.at(i);
      for (unsigned int j = 0; j < cloud1.size(); ++j)
      {
        pcl::PointXYZ pt1 = cloud1.at(j);
        if (dist(pt0, pt1) < cloud_intersect_thresh_) return true;
      }
    }
    return false;
  }

  /**
   * Filter a point cloud to only be above the estimated table and within the
   * workspace in x, then downsample the voxels.
   *
   * @param cloud_in The cloud to filter and downsample
   * @param pub_cloud Publish a cloud of the result if true
   *
   * @return The downsampled cloud
   */
  XYZPointCloud downsampleCloud(XYZPointCloud& cloud_in, bool pub_cloud=false)
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
    if (pub_cloud)
    {
      sensor_msgs::PointCloud2 cloud_down_msg;
      pcl::toROSMsg(cloud_down, cloud_down_msg);
      pcl_down_pub_.publish(cloud_down_msg);
    }
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
  cv::Mat projectProtoObjectsIntoImage(ProtoObjects& objs, cv::Size img_size,
                                       std::string target_frame,
                                       cv::Mat& coord_img)
  {
    cv::Mat obj_img(img_size, CV_8UC1, cv::Scalar(0));
    for (unsigned int i = 0; i < objs.size(); ++i)
    {
      projectPointCloudIntoImage(objs[i].cloud, obj_img,
                                 cur_camera_header_.frame_id, coord_img, i+1);
    }

    return obj_img;
  }

  /**
   * Visualization function of proto objects projected into an image
   *
   * @param obj_img The projected objects image
   * @param objs The set of proto objects
   */
  void displayObjectImage(cv::Mat& obj_img,
                          std::string win_name="projected objects",
                          bool only_moved=false)
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
    cv::imshow(win_name, obj_disp_img);
  }

  void displayColors()
  {
    cv::Mat color_img(250, 200, CV_32FC3);
    for (int r = 0; r < color_img.rows; ++r)
    {
      for (int c = 0; c < color_img.cols; ++c)
      {
        color_img.at<cv::Vec3f>(r,c) = colors_[r/25];
      }
    }
    cv::imshow("color ids", color_img);
  }

  void projectPointCloudIntoImage(XYZPointCloud& cloud, cv::Mat& lbl_img,
                                  std::string target_frame, cv::Mat& coord_img,
                                  unsigned int id=1)
  {
    for (unsigned int i = 0; i < cloud.size(); ++i)
    {
      cv::Point img_idx = projectPointIntoImage(cloud.at(i),
                                                cloud.header.frame_id,
                                                target_frame);
      lbl_img.at<uchar>(img_idx.y, img_idx.x) = id;
      cv::Vec3f coord;
      coord[0] = cloud.at(i).x;
      coord[1] = cloud.at(i).y;
      coord[2] = cloud.at(i).z;
      coord_img.at<cv::Vec3f>(img_idx.y, img_idx.x) = coord;
    }
  }

  cv::Point projectPointIntoImage(pcl::PointXYZ cur_point_pcl,
                                  std::string point_frame,
                                  std::string target_frame)
  {
    PointStamped cur_point;
    cur_point.header.frame_id = point_frame;
    cur_point.point.x = cur_point_pcl.x;
    cur_point.point.y = cur_point_pcl.y;
    cur_point.point.z = cur_point_pcl.z;
    return projectPointIntoImage(cur_point, target_frame);
  }

  cv::Point projectPointIntoImage(PointStamped cur_point)
  {
    return projectPointIntoImage(cur_point, cur_camera_header_.frame_id);
  }

  cv::Point projectPointIntoImage(PointStamped cur_point,
                                  std::string target_frame)
  {
    cv::Point img_loc;
    try
    {
      // Transform point into the camera frame
      PointStamped image_frame_loc_m;
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
      // ROS_ERROR_STREAM(e.what());
    }
    return img_loc;
  }

 protected:
  FeatureTracker* ft_;
  tf::TransformListener* tf_;
  Eigen::Vector4f table_centroid_;
  std::vector<cv::Vec3f> colors_;

 public:
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
  ros::Publisher pcl_obj_seg_pub_;
  ros::Publisher pcl_down_pub_;
  int num_downsamples_;
  sensor_msgs::CameraInfo cam_info_;
  std_msgs::Header cur_camera_header_;
  // XYZPointCloud concave_hull_;
};

class PushOpt
{
 public:
  PushOpt(ProtoTabletopObject& _obj, double _push_angle,
          Eigen::Vector3f _push_vec, unsigned int _id, double _push_dist=0.1) :
      obj(_obj), push_angle(_push_angle), push_unit_vec(_push_vec), id(_id),
      push_dist(_push_dist)
  {
  }
  ProtoTabletopObject obj;
  double push_angle;
  Eigen::Vector3f push_unit_vec;
  unsigned int id;
  double push_dist;

  Eigen::Vector4f getMovedCentroid()
  {
    Eigen::Vector4f new_cent;
    new_cent[0] = obj.centroid[0] + push_unit_vec[0]*push_dist;
    new_cent[1] = obj.centroid[1] + push_unit_vec[1]*push_dist;
    new_cent[2] = obj.centroid[2] + push_unit_vec[2]*push_dist;
    new_cent[3] = 1.0f;
    return new_cent;
  }
};


class PushSample
{
 public:
  unsigned int id;
  double weight;
  double cdf_weight;

  static bool compareSamples(PushSample a, PushSample b)
  {
    return a.weight > b.weight;
  }

  static int cdfBinarySearch(std::vector<PushSample>& scores, float cdf_goal)
  {
    int min_idx = 0;
    int max_idx = scores.size();
    int cur_idx = min_idx + max_idx / 2;
    // NOTE: Assumse scores is sorted in decresaing order
    while (min_idx != max_idx)
    {
      cur_idx = (min_idx + max_idx)/2;
      float cur_val = scores[cur_idx].cdf_weight;
      if (cur_val == cdf_goal || (cur_val > cdf_goal &&
                                  scores[cur_idx+1].cdf_weight < cdf_goal))
      {
        return cur_idx;
      }
      else if (cur_val > cdf_goal)
      {
        min_idx = cur_idx;
      }
      else
      {
        max_idx = cur_idx;
      }
    }
    return cur_idx;
  }

};

typedef std::vector<PushSample> SampleList;

class LinkEdges
{
 public:
  static std::vector<Boundary> edgeLink(cv::Mat& edge_img_raw,
                                        unsigned int min_length=1)
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
    std::vector<Boundary> edges;
    int edge_no = 0;
    for (int r = 0; r < edge_img.rows; ++r)
    {
      for (int c = 0; c < edge_img.cols; ++c)
      {
        if (edge_img_f.at<float>(r,c) == 1)
        {
          Boundary b = trackEdge(edge_img_f, r, c, edge_no++, junctions);
          // Remove short edges
          if (b.size() < min_length) continue;
          edges.push_back(b);
        }
      }
    }
    edge_img_f = -1*edge_img_f;

#ifdef DISPLAY_LINKED_EDGES
    ROS_DEBUG_STREAM("Found " << edges.size() << " edges ");
    cv::Mat edge_disp_img(edge_img.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
    for (unsigned int i = 0; i < edges.size(); ++i)
    {
      cv::Vec3f rand_color;
      rand_color[0] = randf();
      rand_color[1] = randf();
      rand_color[2] = randf();

      for (unsigned int j = 0; j < edges[i].size(); ++j)
      {
        edge_disp_img.at<cv::Vec3f>(edges[i][j].y, edges[i][j].x) = rand_color;
      }
    }
    cv::imshow("linked edges", edge_disp_img);
#endif // DISPLAY_LINKED_EDGES

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

  static Boundary trackEdge(cv::Mat& edge_img, int r_start, int c_start,
                            int edge_no, cv::Mat& junctions)
  {
    Boundary b;
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

class ObjectSingulation
{
 public:
  ObjectSingulation(FeatureTracker* ft, PointCloudSegmentation* pcl_segmenter) :
      ft_(ft), pcl_segmenter_(pcl_segmenter), callback_count_(0), next_id_(0)
  {
    // Create derivative kernels for edge calculation
    cv::getDerivKernels(dy_kernel_, dx_kernel_, 1, 0, CV_SCHARR, true, CV_32F);
    cv::flip(dy_kernel_, dy_kernel_, -1);
    cv::transpose(dy_kernel_, dx_kernel_);
  }

  /**
   * Determine the pushing pose and direction to verify separate objects
   *
   * @param color_img The current color image
   * @param depth_img The current depth image
   * @param cloud     The current point cloud
   * @param workspace_mask The current workspace mask
   *
   * @return The location and orientation to push
   */
  PushVector getPushVector(double push_dist, cv::Mat& color_img,
                           cv::Mat& depth_img, XYZPointCloud& cloud,
                           cv::Mat& workspace_mask)
  {
    ProtoObjects objs = calcProtoObjects(cloud);
    prev_proto_objs_ = cur_proto_objs_;
    cur_proto_objs_ = objs;
    cv::Mat boundary_img;
    std::vector<Boundary> boundaries = getObjectBoundaryStrengths(
        color_img, depth_img, workspace_mask, boundary_img);
    PushVector push_vector = determinePushPose(push_dist, boundary_img, objs,
                                               boundaries);
    ++callback_count_;
    push_vector.num_objects = cur_proto_objs_.size();
    return push_vector;
  }

  /**
   * Randomly choose an object to push that is within reach.
   * Chooses a random direction to push from.
   *
   * @param input_cloud Point cloud containing the tabletop scene to push in
   *
   * @return The location and direction to push.
   */
  PushVector findRandomPushPose(XYZPointCloud& input_cloud)
  {
    ProtoObjects objs = pcl_segmenter_->findTabletopObjects(input_cloud);
    prev_proto_objs_ = cur_proto_objs_;
    cur_proto_objs_ = objs;

    ROS_INFO_STREAM("Found " << objs.size() << " objects.");

    std::vector<int> pushable_obj_idx;
    for (unsigned int i = 0; i < objs.size(); ++i)
    {
      if (objs[i].centroid[0] > min_pushing_x_ &&
          objs[i].centroid[0] < max_pushing_x_ &&
          objs[i].centroid[1] > min_pushing_y_ &&
          objs[i].centroid[1] < max_pushing_y_)
      {
        pushable_obj_idx.push_back(i);
      }
    }
    PushVector p;
    p.header.frame_id = workspace_frame_;

    if (pushable_obj_idx.size() < 1)
    {
      ROS_WARN_STREAM("No object clusters found! Returning empty push_pose");
      return p;
    }
    ROS_INFO_STREAM("Found " << pushable_obj_idx.size()
                    << " pushable proto objects");
    int rand_idx = pushable_obj_idx[rand() % pushable_obj_idx.size()];
    Eigen::Vector4f obj_xyz_centroid = objs[rand_idx].centroid;
    p.start_point.x = obj_xyz_centroid[0];
    p.start_point.y = obj_xyz_centroid[1];
    // Set z to be the table height
    p.start_point.z = objs[0].table_centroid[2];

    sensor_msgs::PointCloud2 obj_push_msg;
    pcl::toROSMsg(objs[rand_idx].cloud, obj_push_msg);
    obj_push_pub_.publish(obj_push_msg);

    // Choose a random orientation
    double rand_orientation = (randf()*
                               (max_push_angle_- min_push_angle_) +
                               min_push_angle_);
    ROS_INFO_STREAM("Chosen push pose is at: (" << obj_xyz_centroid[0] << ", "
                    << obj_xyz_centroid[1] << ", " << objs[0].table_centroid[2]
                    << ") with orientation of: " << rand_orientation);
    p.push_angle = rand_orientation;
    return p;
  }

 protected:
  /**
   * Find the current object estimates in the current cloud, dependent on the
   * previous cloud
   *
   * @param cloud The cloud to find the objects in.
   *
   * @return The current estimate of the objects
   */
  ProtoObjects calcProtoObjects(XYZPointCloud& cloud)
  {
    // pcl_segmenter_->displayColors();
    XYZPointCloud objs_cloud;
    ProtoObjects objs = pcl_segmenter_->findTabletopObjects(cloud, objs_cloud,
                                                            true);
    publishObjects(objs);
    XYZPointCloud cur_objs_down = pcl_segmenter_->downsampleCloud(objs_cloud,
                                                                  true);
    ProtoObjects cur_objs;
    if (callback_count_ > 1)
    {
      // Determine where stuff has moved
      ProtoObjects moved_regions = pcl_segmenter_->getMovedRegions(
          prev_objs_down_, cur_objs_down);
      // Match these moved regions to the previous objects
      pcl_segmenter_->matchMovedRegions(prev_proto_objs_, moved_regions);
      // Match the moved objects to their new locations
      updateMovedObjs(objs, prev_proto_objs_);

#ifdef DISPLAY_PREV_PROJECTED_OBJECTS
      cv::Mat prev_obj_coords(cv::Size(320,240), CV_32FC3, cv::Scalar(0,0,0));
      cv::Mat prev_obj_img = pcl_segmenter_->projectProtoObjectsIntoImage(
          prev_proto_objs_, prev_obj_coords.size(), workspace_frame_,
          prev_obj_coords);
      pcl_segmenter_->displayObjectImage(prev_obj_img, "prev objects");
#endif // DISPLAY_PREV_PROJECTED_OBJECTS
    }
    else
    {
      // Initialize IDs
      for (unsigned int i = 0; i < objs.size(); ++i)
      {
        objs[i].id = getNextID();
      }
    }
    prev_objs_down_ = cur_objs_down;
    return objs;
  }

    /**
   * Method to update the IDs and of moved proto objects
   *
   * @param cur_objs The current set of proto objects
   * @param prev_objs The previous set of proto objects
   * @param moved_objs Moved proto objects
   *
   * @return
   */
  void updateMovedObjs(ProtoObjects& cur_objs, ProtoObjects& prev_objs)
  {
    const bool merged = cur_objs.size()  < prev_objs.size();
    const bool split = cur_objs.size()  > prev_objs.size();
    if (merged)
    {
      ROS_WARN_STREAM("Objects merged from " << prev_objs.size() << " to " <<
                      cur_objs.size());
      // TODO: Something different for merging, check which moved objects
      // combined with which unmoved objects
    }
    else if (split)
    {
      ROS_WARN_STREAM("Objects split from " << prev_objs.size() << " to " <<
                      cur_objs.size());
      // TODO: Deal with adding new object ids for splitting, create a
      // static / global id generator method?
    }
    else
    {
      ROS_INFO_STREAM("Same number of objects: " << prev_objs.size());
    }

    std::vector<bool> matched = matchUnmoved(cur_objs, prev_objs);
    matchMoved(cur_objs, prev_objs, matched);
  }

  std::vector<bool> matchUnmoved(ProtoObjects& cur_objs,
                                 ProtoObjects& prev_objs)
  {
    std::vector<bool> matched(cur_objs.size(), false);
    // First match the unmoved objects
    for (unsigned int i = 0; i < prev_objs.size(); ++i)
    {
      if (!prev_objs[i].moved)
      {
        double min_score = FLT_MAX;
        unsigned int min_idx = cur_objs.size();
        // Update the ID in cur_objs of the closest centroid in previous objects
        for (unsigned int j = 0; j < cur_objs.size(); ++j)
        {
          double score = pcl_segmenter_->sqrDist(prev_objs[i].centroid,
                                                 cur_objs[j].centroid);
          if (score < min_score)
          {
            min_idx = j;
            min_score = score;
          }
        }
        if (min_idx < cur_objs.size())
        {
          // TODO: Ensure uniquness
          ROS_INFO_STREAM("Prev unmoved obj: " << prev_objs[i].id << ", " << i
                          << " maps to cur " << min_idx << " : " << min_score);
          if (!matched[min_idx])
          {
            cur_objs[min_idx].id = prev_objs[i].id;
            cur_objs[min_idx].delta_c = (cur_objs[min_idx].centroid -
                                         prev_objs[i].centroid);
          }

          matched[min_idx] = true;
        }
      }
    }
    return matched;
  }

  void matchMoved(ProtoObjects& cur_objs, ProtoObjects& prev_objs,
                  std::vector<bool> matched)
  {
    for (unsigned int i = 0; i < prev_objs.size(); ++i)
    {
      if (prev_objs[i].moved)
      {
        double min_score = std::numeric_limits<double>::max();
        unsigned int min_idx = cur_objs.size();
        // Match the moved objects to their new locations
        ROS_DEBUG_STREAM("Finding match for object : " << prev_objs[i].id);
        for (unsigned int j = 0; j < cur_objs.size(); ++j)
        {
          if (!matched[j])
          {
            // Run ICP to match between frames
            double cur_score = pcl_segmenter_->ICPProtoObjects(prev_objs[i],
                                                               cur_objs[j]);
            if (cur_score < min_score)
            {
              min_score = cur_score;
              min_idx = j;
            }
          }
        }
        if (min_idx < cur_objs.size())
        {
          // TODO: If score is too bad ignore
          ROS_INFO_STREAM("Prev moved obj: " << prev_objs[i].id  << ", " << i
                          << " maps to cur " << min_idx << " : " << min_score);
          if (matched[min_idx])
          {
          }
          else
          {
            cur_objs[min_idx].id = prev_objs[i].id;
            cur_objs[min_idx].delta_c = (cur_objs[min_idx].centroid -
                                         prev_objs[i].centroid);
            matched[min_idx] = true;
          }

        }
        else
        {
          ROS_WARN_STREAM("No match for moved previus object: "
                          << prev_objs[i].id);
        }
      }
      for (unsigned int i = 0; i < matched.size(); ++i)
      {
        if (!matched[i]) cur_objs[i].id = getNextID();
      }
    }
  }


  /**
   * Determine what push to make given the current object and boundary estimates
   *
   * @param boundary_img The image of the estimated boundary strengths
   * @param objs The estimated set of proto objects
   *
   * @return A push for the robot to make to singulate objects
   */
  PushVector determinePushPose(double push_dist, cv::Mat& boundary_img,
                               ProtoObjects& objs,
                               std::vector<Boundary>& boundaries)
  {
    cv::Mat obj_coords(boundary_img.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
    cv::Mat obj_lbl_img = pcl_segmenter_->projectProtoObjectsIntoImage(
        objs, boundary_img.size(), workspace_frame_, obj_coords);

    get3DBoundaries(boundaries, obj_coords);
    associate3DBoundaries(boundaries, objs, obj_lbl_img);

#ifdef DISPLAY_PROJECTED_OBJECTS
    pcl_segmenter_->displayObjectImage(obj_lbl_img);
#endif // DISPLAY_PROJECTED_OBJECTS
#ifdef DISPLAY_3D_BOUNDARIES
    draw3DBoundaries(boundaries, obj_lbl_img, objs.size());
#endif // DISPLAY_3D_BOUNDARIES

    Boundary test_boundary = chooseTestBoundary(boundaries, objs, obj_lbl_img);

    if (test_boundary.object_id == objs.size())
    {
      ROS_WARN_STREAM("Boundary has no ID!");
      PushVector push_pose;
      return push_pose;
    }
    return determinePushVector(push_dist, test_boundary, objs, obj_lbl_img,
                               obj_coords);
  }

  /**
   * Determine how the robot should push to disambiguate the given boundary
   * hypotheses
   *
   * @param hypo_img The image containing the boundary hypothesis
   * @param objs The set of objects
   * @param obj_img The objects projected into the image frame
   *
   * @return The push command
   */
  PushVector determinePushVector(double push_dist, Boundary& boundary,
                                 ProtoObjects& objs, cv::Mat& obj_lbl_img,
                                 cv::Mat& obj_coords)
  {
#ifdef DISPLAY_CHOSEN_BOUNDARY
    displayBoundaryImage(obj_lbl_img, boundary, "chosen", true);
#endif // DISPLAY_CHOSEN_BOUNDARY

    // Split point cloud at location of the boundary
    ProtoObjects split_objs3D = splitObject3D(boundary, objs[boundary.
                                                             object_id]);
    cv::Mat split_img = pcl_segmenter_->projectProtoObjectsIntoImage(
        split_objs3D, obj_lbl_img.size(), workspace_frame_, obj_coords);
    pcl_segmenter_->displayObjectImage(split_img, "3D Split");

    // TODO: Generalize to more than 2 object splits
    PushVector push_pose = getPushDirection(push_dist, split_objs3D[0],
                                            split_objs3D[1], objs,
                                            boundary.object_id,
                                            obj_lbl_img);
    push_pose.header.frame_id = workspace_frame_;
    return push_pose;
  }

  /**
   * Determine the strength of object boundaries in an RGB-D image
   *
   * @param color_img The color image
   * @param depth_img The depth image
   * @param workspace_mask A mask depicting locations of interest
   * @param combined_edges Edge image
   *
   * @return A vector of the detected Boundary objects
   */
  std::vector<Boundary> getObjectBoundaryStrengths(cv::Mat& color_img,
                                                   cv::Mat& depth_img,
                                                   cv::Mat& workspace_mask,
                                                   cv::Mat& combined_edges)
  {

    cv::Mat tmp_bw(color_img.size(), CV_8UC1);
    cv::Mat bw_img(color_img.size(), CV_32FC1);
    cv::Mat Ix(bw_img.size(), CV_32FC1);
    cv::Mat Iy(bw_img.size(), CV_32FC1);
    cv::Mat Ix_d(bw_img.size(), CV_32FC1);
    cv::Mat Iy_d(bw_img.size(), CV_32FC1);
    cv::Mat edge_img(color_img.size(), CV_32FC1);
    cv::Mat depth_edge_img(color_img.size(), CV_32FC1);
    cv::Mat edge_img_masked(edge_img.size(), CV_32FC1, cv::Scalar(0.0));
    cv::Mat depth_edge_img_masked(edge_img.size(), CV_32FC1, cv::Scalar(0.0));

    // Convert to grayscale
    cv::cvtColor(color_img, tmp_bw, CV_BGR2GRAY);
    tmp_bw.convertTo(bw_img, CV_32FC1, 1.0/255);

    // Get image derivatives
    cv::filter2D(bw_img, Ix, CV_32F, dx_kernel_);
    cv::filter2D(bw_img, Iy, CV_32F, dy_kernel_);
    cv::filter2D(depth_img, Ix_d, CV_32F, dx_kernel_);
    cv::filter2D(depth_img, Iy_d, CV_32F, dy_kernel_);

    // Create magintude image
    for (int r = 0; r < edge_img.rows; ++r)
    {
      float* mag_row = edge_img.ptr<float>(r);
      float* Ix_row = Ix.ptr<float>(r);
      float* Iy_row = Iy.ptr<float>(r);
      for (int c = 0; c < edge_img.cols; ++c)
      {
        mag_row[c] = sqrt(Ix_row[c]*Ix_row[c] + Iy_row[c]*Iy_row[c]);
      }
    }
    for (int r = 0; r < depth_edge_img.rows; ++r)
    {
      float* mag_row = depth_edge_img.ptr<float>(r);
      float* Ix_row = Ix_d.ptr<float>(r);
      float* Iy_row = Iy_d.ptr<float>(r);
      for (int c = 0; c < depth_edge_img.cols; ++c)
      {
        mag_row[c] = sqrt(Ix_row[c]*Ix_row[c] + Iy_row[c]*Iy_row[c]);
      }
    }

    // Remove stuff from the image
    edge_img.copyTo(edge_img_masked, workspace_mask);
    depth_edge_img.copyTo(depth_edge_img_masked, workspace_mask);
    if (threshold_edges_)
    {
      cv::Mat bin_depth_edges;
      cv::threshold(depth_edge_img_masked, bin_depth_edges,
                    depth_edge_weight_thresh_, depth_edge_weight_,
                    cv::THRESH_BINARY);
      cv::Mat bin_img_edges;
      cv::threshold(edge_img_masked, bin_img_edges, edge_weight_thresh_,
                    (1.0-depth_edge_weight_), cv::THRESH_BINARY);
      combined_edges = bin_depth_edges + bin_img_edges;
      double edge_max = 1.0;
      double edge_min = 1.0;
      cv::minMaxLoc(edge_img_masked, &edge_min, &edge_max);
      double depth_max = 1.0;
      double depth_min = 1.0;
      cv::minMaxLoc(depth_edge_img_masked, &depth_min, &depth_max);
    }
    else
    {
      if (use_weighted_edges_)
      {
        combined_edges = (edge_img_masked*(1.0-depth_edge_weight_) +
                          depth_edge_weight_*depth_edge_img_masked);
      }
      else
      {
        combined_edges = cv::max(edge_img_masked, depth_edge_img_masked);
      }
    }

    // Link edges into object boundary hypotheses
    std::vector<Boundary> boundaries = LinkEdges::edgeLink(combined_edges,
                                                           min_edge_length_);
    // TODO: Hypothesize boundary likelihoods with learned function from a
    // combination of cues

    return boundaries;
  }

  void get3DBoundaries(std::vector<Boundary>& boundaries, cv::Mat& obj_coords)
  {
    for (unsigned int b = 0; b <boundaries.size(); ++b)
    {
      get3DBoundary(boundaries[b], obj_coords);
    }
  }

  void get3DBoundary(Boundary& b, cv::Mat& obj_coords)
  {
    b.points3D.clear();
    for (unsigned int i = 0; i < b.size();++i)
    {
      cv::Vec3f p = obj_coords.at<cv::Vec3f>(b[i].y, b[i].x);
      // Don't add empty points
      if (p[0] == 0.0 && p[1] == 0.0 && p[2] == 0.0) continue;
      b.points3D.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
    }
  }

  // TODO: Need to match previous and current boundaries
  void associate3DBoundaries(std::vector<Boundary>& boundaries,
                             ProtoObjects& objs, cv::Mat& obj_lbl_img)
  {
    int no_overlap_count = 0;
    for (unsigned int b = 0; b < boundaries.size(); ++b)
    {
      std::vector<int> obj_overlaps(objs.size(), 0);
      for (unsigned int i = 0; i < boundaries[b].size(); ++i)
      {
        unsigned int id = obj_lbl_img.at<uchar>(boundaries[b][i].y,
                                                boundaries[b][i].x);
        if (id > 0)
        {
          obj_overlaps[id-1]++;
        }
      }
      int max_overlap = 0;
      unsigned int max_id = objs.size();
      for (unsigned int o = 0; o < objs.size(); ++o)
      {
        if (obj_overlaps[o] > max_overlap)
        {
          max_overlap = obj_overlaps[o];
          max_id = o;
        }
      }
      if (max_id == objs.size())
      {
        ROS_DEBUG_STREAM("No overlap for boundary: " << b);
        boundaries[b].object_id = objs.size();
        no_overlap_count++;
      }
      else
      {
        boundaries[b].object_id = max_id;
        // TODO: Here is a place the ID depends on the image id
        objs[max_id].boundaries.push_back(boundaries[b]);
      }
    }
    ROS_DEBUG_STREAM("No overlap for: " << no_overlap_count << " of " <<
                     boundaries.size() << " boundaries");
  }

  // Get plane best containing the 3D curve
  Eigen::Vector4f planeRANSAC(Boundary& b)
  {
    XYZPointCloud cloud;
    cloud.resize(b.points3D.size());
    for (unsigned int i = 0; i < b.points3D.size(); ++i)
    {
      cloud.at(i) = b.points3D[i];
    }
    if (b.points3D.size() < 4)
    {
      Eigen::Vector4f unit_norm;
      ROS_WARN_STREAM("Plane has 3 or fewer points.");
      if (b.points3D.size() == 3)
      {
        ROS_WARN_STREAM("Plane has exactly 3 points.");
      }
      if (b.points3D.size() == 2)
      {
        ROS_WARN_STREAM("Will get plane running through line and origin.");
      } else if (b.points3D.size() > 0)
      {
        ROS_WARN_STREAM("Will get plane running through point and z-axis.");
      }
      else
      {
        ROS_WARN_STREAM("No 3D points");
      }
      return unit_norm;
    }

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices plane_inliers;
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(boundary_ransac_thresh_);
    plane_seg.setInputCloud(cloud.makeShared());
    plane_seg.segment(plane_inliers, coefficients);
    ROS_DEBUG_STREAM("Plane Hessian Normal Form: (" << coefficients.values[0] <<
                     ", " << coefficients.values[1] << ", " <<
                     coefficients.values[2] << ", " << coefficients.values[3] <<
                     ")");

    Eigen::Vector4f hessian;
    hessian[0] = coefficients.values[0];
    hessian[1] = coefficients.values[1];
    hessian[2] = coefficients.values[2];
    hessian[3] = coefficients.values[3];
    return hessian;
  }


  Boundary chooseTestBoundary(std::vector<Boundary>& boundaries,
                              ProtoObjects& objs, cv::Mat& obj_img)
  {
    std::vector<double> scores(boundaries.size(), 0.0f);
    for (unsigned int i = 0; i < boundaries.size(); ++i)
    {
      if (boundaries[i].object_id >= objs.size() ||
          boundaries[i].points3D.size() < 4)
      {
        continue;
      }
      // NOTE: Currently choose on ratio of split object sizes
      // TODO: Should use estimated internal and external likelihoods
      ProtoObjects split = splitObject3D(boundaries[i],
                                         objs[boundaries[i].object_id]);
      double s0 = split[0].cloud.size();
      double s1 = split[1].cloud.size();
      if (s0 == 0) scores[i] = 0;
      else if (s1 == 0) scores[i] = 0;
      else if (s0 > s1) scores[i] = s1/s0;
      else scores[i] = s0/s1;
    }
    unsigned int chosen_id = sampleScore(scores);
    ROS_DEBUG_STREAM("Chose boundary: " << chosen_id << " has objet_id: " <<
                     boundaries[chosen_id].object_id << " and " <<
                     boundaries[chosen_id].points3D.size() << " 3D points.");
    return boundaries[chosen_id];
  }

  /**
   * Determine the best direction to push given the current set of objects and
   * the hypothesized splits.
   *
   * @param split0 The first hypothesized object split
   * @param split1 The second hypothesized object split
   * @param objs The current set of object estimates
   * @param id The ID of the object that is trying to the splits
   *
   * @return The push to make
   */
  PushVector getPushDirection(double push_dist,
                              ProtoTabletopObject& split0,
                              ProtoTabletopObject& split1,
                              ProtoObjects& objs, unsigned int id,
                              cv::Mat& lbl_img)
  {
    // Get vector between the two split object centroids and find the normal to
    // the vertical plane running through this vector
    const Eigen::Vector4f split_diff = split0.centroid - split1.centroid;
    const Eigen::Vector3f split_diff3(split_diff[0], split_diff[1],
                                      split_diff[2]);
    const Eigen::Vector3f z_axis(0, 0, 1);
    const Eigen::Vector3f x_axis(1, 0, 0);
    const Eigen::Vector3f split_norm = split_diff3.cross(z_axis);
    const Eigen::Vector3f push_vec_pos = split_norm/split_norm.norm();
    const Eigen::Vector3f push_vec_neg = -split_norm/split_norm.norm();
    const double push_angle_pos = std::atan2(push_vec_pos[1], push_vec_pos[0]);
    const double push_angle_neg = std::atan2(push_vec_neg[1], push_vec_neg[0]);
    // Find the highest clearance push from the four possible combinations of
    // split object and angles
    std::vector<PushOpt> split_opts;
    split_opts.push_back(PushOpt(split0, push_angle_pos, push_vec_pos, id,
                                 push_dist));
    split_opts.push_back(PushOpt(split0, push_angle_neg, push_vec_neg, id,
                                 push_dist));
    split_opts.push_back(PushOpt(split1, push_angle_pos, push_vec_pos, id,
                                 push_dist));
    split_opts.push_back(PushOpt(split1, push_angle_neg, push_vec_neg, id,
                                 push_dist));
    double max_clearance = 0.0;
    unsigned int max_id = split_opts.size();
    for (unsigned int i = 0; i < split_opts.size(); ++i)
    {
      if (split_opts[i].push_angle < min_push_angle_ ||
          split_opts[i].push_angle > max_push_angle_) continue;
      const double clearance = getSplitPushClearance(split_opts[i], objs);
      if (clearance > max_clearance)
      {
        max_clearance = clearance;
        max_id = i;
      }
    }
    if (max_id == split_opts.size())
    {
      // NOTE: Nothing found
      ROS_WARN_STREAM("No object push found!");
      PushVector push;
      push.start_point.x = 0.0;
      push.start_point.y = 0.0;
      push.start_point.z = 0.0;
      push.push_angle = 0.0;
      return push;
    }
    PushVector push;
    push.start_point.x = split_opts[max_id].obj.centroid[0];
    push.start_point.y = split_opts[max_id].obj.centroid[1];
    push.start_point.z = split_opts[max_id].obj.centroid[2];
    push.push_angle = split_opts[max_id].push_angle;

#ifdef DISPLAY_PROJECTED_OBJECTS

    const Eigen::Vector4f moved_cent = split_opts[max_id].getMovedCentroid();
    PointStamped start_point;
    start_point.point = push.start_point;
    start_point.header.frame_id = workspace_frame_;
    PointStamped end_point;
    end_point.point.x = moved_cent[0];
    end_point.point.y = moved_cent[1];
    end_point.point.z = moved_cent[2];
    end_point.header.frame_id = workspace_frame_;

    cv::Point img_start_point = pcl_segmenter_->projectPointIntoImage(start_point);
    cv::Point img_end_point = pcl_segmenter_->projectPointIntoImage(end_point);
    cv::Mat obj_img_f;
    cv::Mat disp_img(lbl_img.size(), CV_32FC3);
    lbl_img.convertTo(obj_img_f, CV_32FC1, 30.0/255);
    cv::cvtColor(obj_img_f, disp_img, CV_GRAY2BGR);
    cv::line(disp_img, img_start_point, img_end_point, cv::Scalar(0,0,1.0));
    cv::circle(disp_img, img_end_point, 4, cv::Scalar(0,1.0,0));
    cv::imshow("push_vector", disp_img);
#endif // DISPLAY_PROJECTED_OBJECTS

    return push;
  }

  /**
   * Aproximate the amount of clearance between objects after performing the push
   *
   * @param split The split object to push
   * @param objs The current set of object estimates
   *
   * @return The clearance in meters
   */
  double getSplitPushClearance(PushOpt& split, ProtoObjects& objs)
  {
    // TODO: Check angle stuff
    double min_clearance = FLT_MAX;
    const Eigen::Vector4f moved_cent = split.getMovedCentroid();
    for (unsigned int i = 0; i < objs.size(); ++i)
    {
      // Don't compare object to itself
      if (i == split.id) continue;

      const double clearance = (moved_cent - objs[i].centroid).norm();

      if (clearance < min_clearance)
      {
        min_clearance = clearance;
      }
    }
    return min_clearance;
  }

  ProtoObjects splitObject3D(Boundary& boundary, ProtoTabletopObject& to_split)
  {
    // Get plane containing the boundary
    Eigen::Vector4f hessian = planeRANSAC(boundary);
    // Split based on the plane
    return splitObject3D(hessian, to_split);
  }

  ProtoObjects splitObject3D(Eigen::Vector4f& hessian,
                             ProtoTabletopObject& to_split)
  {
    const float denom = std::sqrt(hessian[0]*hessian[0] + hessian[1]*hessian[1]
                                  + hessian[2]*hessian[2]);
    const Eigen::Vector3f norm(hessian[0]/denom, hessian[1]/denom,
                               hessian[2]/denom);
    const float p = hessian[3]/denom;

    // Split the point clouds based on the half plane distance test
    pcl::PointIndices p1;
    pcl::PointIndices p2;
    for (unsigned int i = 0; i < to_split.cloud.size(); ++i)
    {
      const pcl::PointXYZ x = to_split.cloud.at(i);
      const float d = norm[0]*x.x + norm[1]*x.y + norm[2]*x.z + p;
      if (d > 0)
      {
        p1.indices.push_back(i);
      }
      else
      {
        p2.indices.push_back(i);
      }
    }
    ROS_DEBUG_STREAM("split3D: P1 has " << p1.indices.size() << " points");
    ROS_DEBUG_STREAM("split3D: P2 has " << p2.indices.size() << " points");
    // Extract indices
    ProtoObjects split;
    ProtoTabletopObject po1;
    ProtoTabletopObject po2;
    // TODO: Fix ID stuff
    // po1.id = to_split.id;
    // po2.id = to_split.id;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices plane_outliers;
    extract.setInputCloud(to_split.cloud.makeShared());
    extract.setIndices(boost::make_shared<pcl::PointIndices>(p1));
    extract.filter(po1.cloud);
    extract.setNegative(true);
    extract.filter(po2.cloud);
    split.push_back(po1);
    split.push_back(po2);
    for (unsigned int i = 0; i < split.size(); ++i)
    {
      pcl::compute3DCentroid(split[i].cloud, split[i].centroid);
    }

    return split;
  }


  /**
   * Splits an object into two new objects based on the passed in boundary.
   *
   * @param obj_lbl_img Image of object labels
   * @param boundary The boundary on which to split
   * @param obj_coords The image of object 3D-coordinates
   * @param id The image id
   *
   * @return The two new objects after the split
   */
  ProtoObjects splitObject(cv::Mat& obj_lbl_img, Boundary& boundary,
                           cv::Mat& obj_coords, unsigned int id)
  {
    cv::Mat split_img = splitObjectImage(obj_lbl_img, boundary, id);
#ifdef DISPLAY_OBJECT_SPLITS
    cv::imshow("split_objs", split_img*60);
#endif // DISPLAY_OBJECT_SPLITS
    ProtoObjects split;
    ProtoTabletopObject po1;
    ProtoTabletopObject po2;
    int po1_count = 0;
    int po2_count = 0;
    for (int r = 0; r < split_img.rows; ++r)
    {
      for (int c = 0; c < split_img.cols; ++c)
      {
        if (split_img.at<uchar>(r,c) == 1) ++po1_count;
        else if (split_img.at<uchar>(r,c) == 2) ++po2_count;
      }
    }
    po1.cloud.resize(po1_count);
    po2.cloud.resize(po2_count);
    po1_count = 0;
    po2_count = 0;
    for (int r = 0; r < split_img.rows; ++r)
    {
      for (int c = 0; c < split_img.cols; ++c)
      {
        if (split_img.at<uchar>(r,c) == 1)
        {
          pcl::PointXYZ p;
          cv::Vec3f p_i = obj_coords.at<cv::Vec3f>(r,c);
          p.x = p_i[0];
          p.y = p_i[1];
          p.z = p_i[2];
          po1.cloud.at(po1_count++) = p;
        }
        else if (split_img.at<uchar>(r,c) == 2)
        {
          pcl::PointXYZ p;
          cv::Vec3f p_i = obj_coords.at<cv::Vec3f>(r,c);
          p.x = p_i[0];
          p.y = p_i[1];
          p.z = p_i[2];
          po2.cloud.at(po2_count++) = p;
        }
      }
    }
    split.push_back(po1);
    split.push_back(po2);

    for (unsigned int i = 0; i < split.size(); ++i)
    {
      pcl::compute3DCentroid(split[i].cloud, split[i].centroid);
    }

    return split;
  }

  /**
   * Function creates an image holding the correct points on each side of the
   * given boundary extended to hit the image edges
   *
   * @param obj_img The image of objects projected into view
   * @param boundary The boundary location to test
   * @param objs The set of objects
   * @param id The id of the object to test
   *
   * @return The image containing pixels labeled on the two sides of the boundary
   */
  cv::Mat splitObjectImage(cv::Mat& obj_img, Boundary& boundary,
                           unsigned int id)
  {
    cv::Mat sides_img = splitOnLine(boundary, obj_img.size());
    cv::Mat split_objects(sides_img.size(), CV_8UC1, cv::Scalar(0));
    // TODO: Better way of doing this
    // TODO: Fix id to be consistent...
    for (int r = 0; r < split_objects.rows; ++r)
    {
      for (int c = 0; c < split_objects.cols; ++c)
      {
        if (obj_img.at<uchar>(r,c) == id+1)
        {
          if (sides_img.at<uchar>(r,c) == 1)
          {
            split_objects.at<uchar>(r,c) = 1;
          }
          else if (sides_img.at<uchar>(r,c) == 2)
          {
            split_objects.at<uchar>(r,c) = 2;
          }
        }
      }
    }
    return split_objects;
  }

  /**
   * Creates an image with two regions split by the given boundary
   *
   * @param boundary The boundary on which to split
   * @param img_size The image size to use in spliting the image
   *
   * @return The split image
   */
  cv::Mat splitOnLine(Boundary& boundary, cv::Size img_size)
  {
    Boundary lb = extendBoundary(boundary, img_size);
    std::map<int, int> bmap;
    int min_y = img_size.height;
    int max_y = 0;
    int x_at_min_y = 0;
    int x_at_max_y = 0;
    for (unsigned int i = 0; i < lb.size(); ++i)
    {
      bmap[lb[i].y] = lb[i].x;
      if (lb[i].y < min_y)
      {
        min_y = lb[i].y;
        x_at_min_y = lb[i].x;
      }
      if (lb[i].y > max_y)
      {
        max_y = lb[i].y;
        x_at_max_y = lb[i].x;
      }
    }

    cv::Mat side_img(img_size, CV_8UC1, cv::Scalar(0));

    const bool horizontal = boundary[0].y == boundary[2].y;
    const bool vertical = boundary[0].x == boundary[2].x;
    if (horizontal)
    {
      for (int r = 0; r < side_img.rows; ++r)
      {
        if (r == boundary[0].y) continue;
        uchar* row = side_img.ptr<uchar>(r);
        for (int c = 0; c < side_img.cols; ++c)
        {
          if (r < boundary[0].y)
          {
            row[c] = 1;
          }
          else
          {
            row[c] = 2;
          }
        }
      }
    }
    else if (vertical)
    {
      for (int r = 0; r < side_img.rows; ++r)
      {
        uchar* row = side_img.ptr<uchar>(r);
        for (int c = 0; c < side_img.cols; ++c)
        {
          if (c < boundary[0].x)
          {
            row[c] = 1;
          }
          else if (c > boundary[0].x)
          {
            row[c] = 2;
          }
        }
      }
    }
    else
    {
      const bool pos_slope = ((min_y-max_y)/(x_at_min_y-x_at_max_y) > 0);
      for (int r = 0; r < side_img.rows; ++r)
      {
        uchar* row = side_img.ptr<uchar>(r);
        const int line_x = bmap[r];
        for (int c = 0; c < side_img.cols; ++c)
        {
          // NOTE: Deal with clipped lines
          if (r < min_y)
          {
            if (pos_slope)
            {
              row[c] = 2;
            }
            else
            {
              row[c] = 1;
            }
          }
          else if (r > max_y)
          {
            if (pos_slope)
            {
              row[c] = 1;
            }
            else
            {
              row[c] = 2;
            }
          }
          // Regular case
          else if (c < line_x)
          {
            row[c] = 1;
          }
          else if (c > line_x)
          {
            row[c] = 2;
          }
        }
      }
    }
    return side_img;
  }

  /**
   * Method to extend a boundary to the edges of an image
   *
   * @param boundary The line to extend
   * @param img_size Size of the image the line extends in
   *
   * @return The extended line
   */
  Boundary extendBoundary(Boundary& boundary, cv::Size img_size)
  {
    const int x_max = img_size.width - 1;
    const int y_max = img_size.height - 1;
    if (boundary[0].x == boundary[1].x)
    {
      cv::Point p1;
      cv::Point p2;
      p1.x = boundary[0].x;
      p1.y = 0;
      p2.x = boundary[0].x;
      p2.y = y_max;
      return getLineValues(p1, p2, img_size);
    }
    else if (boundary[0].y == boundary[1].y)
    {
      cv::Point p1;
      cv::Point p2;
      p1.x = 0;
      p1.y = boundary[0].y;
      p2.x = x_max;
      p2.y = boundary[0].y;
      return getLineValues(p1, p2, img_size);
    }
    else
    {
      cv::Point p1;
      p1.x = boundary[0].x;
      p1.y = boundary[0].y;
      cv::Point p2;
      p2.x = boundary[2].x;
      p2.y = boundary[2].y;
      float m = (p1.y - p2.y) / (p1.x - p2.x);
      cv::Point left;
      cv::Point right;
      cv::Point top;
      cv::Point bottom;
      left.x = 0;
      right.x = x_max;
      top.y = 0;
      bottom.y = y_max;
      left.y = static_cast<int>(m*(left.x - p1.x) + p1.y);
      right.y = static_cast<int>(m*(right.x - p1.x) + p1.y);
      top.x = static_cast<int>((top.y - p1.y)/m + p1.x);
      bottom.x = static_cast<int>((bottom.y - p1.y)/m + p1.x);
      if (top.x > 0 && top.x < x_max)
      {
        if (bottom.x >= 0 && bottom.x <= x_max)
        {
          return getLineValues(top, bottom, img_size);
        }
        else if (left.y >=0 && left.y <= y_max)
        {
          return getLineValues(top, left, img_size);
        }
        else if (right.y >=0 && right.y <= y_max)
        {
          return getLineValues(top, right, img_size);
        }
      }
      else if (bottom.x >= 0 && bottom.x <= x_max)
      {
        if (left.y >=0 && left.y <= y_max)
        {
          return getLineValues(bottom, left, img_size);
        }
        else if (right.y >=0 && right.y <= y_max)
        {
          return getLineValues(bottom, right, img_size);
        }
      }
      else
      {
        return getLineValues(left, right, img_size);
      }
    }
  }

  Boundary getLineValues(cv::Point p1, cv::Point p2, cv::Size frame_size)
  {
    Boundary line;
    bool steep = (abs(p1.y - p2.y) > abs(p1.x - p2.x));
    if (steep)
    {
      // Swap x and y
      cv::Point tmp(p1.y, p1.x);
      p1.x = tmp.x;
      p1.y = tmp.y;
      tmp.y = p2.x;
      tmp.x = p2.y;
      p2.x = tmp.x;
      p2.y = tmp.y;
    }
    if (p1.x > p2.x)
    {
      // Swap p1 and p2
      cv::Point tmp(p1.x, p1.y);
      p1.x = p2.x;
      p1.y = p2.y;
      p2.x = tmp.x;
      p2.y = tmp.y;
    }
    int dx = p2.x - p1.x;
    int dy = abs(p2.y - p1.y);
    int error = dx / 2;
    int ystep = 0;
    if (p1.y < p2.y)
    {
      ystep = 1;
    }
    else if (p1.y > p2.y)
    {
      ystep = -1;
    }
    for(int x = p1.x, y = p1.y; x <= p2.x; ++x)
    {
      if (steep)
      {
        cv::Point p_new(y,x);
        // Test that p_new is in the image
        if (x < 0 || y < 0 || x >= frame_size.height || y >= frame_size.width ||
            (x == 0 && y == 0))
        {
        }
        else
        {
          line.push_back(p_new);
        }
      }
      else
      {
        cv::Point p_new(x,y);
        // Test that p_new is in the image
        if (x < 0 || y < 0 || x >= frame_size.width || y >= frame_size.height ||
            (x == 0 && y == 0))
        {
        }
        else
        {
          line.push_back(p_new);
        }
      }
      error -= dy;
      if (error < 0)
      {
        y += ystep;
        error += dx;
      }
    }
    return line;
  }

  /**
   * Helper method to publish a set of proto objects for display purposes.
   *
   * @param objs The objects to publish
   */
  void publishObjects(ProtoObjects& objs)
  {
    if (objs.size() == 0) return;
    XYZPointCloud objs_cloud = objs[0].cloud;
    for (unsigned int i = 1; i < objs.size(); ++i)
    {
      objs_cloud += objs[i].cloud;
    }

    pcl::PointCloud<pcl::PointXYZI> label_cloud;
    label_cloud.header.frame_id = objs_cloud.header.frame_id;
    label_cloud.height = objs_cloud.height;
    label_cloud.width = objs_cloud.width;
    label_cloud.resize(label_cloud.height*label_cloud.width);
    for (unsigned int i = 1, j=0; i < objs.size(); ++i)
    {
      for (unsigned int k=0; k < objs[i].cloud.size(); ++k, ++j)
      {
        // TODO: This is not working
        label_cloud.at(j).x = objs_cloud.at(j).x;
        label_cloud.at(j).y = objs_cloud.at(j).y;
        label_cloud.at(j).z = objs_cloud.at(j).z;
        label_cloud.at(j).intensity = objs[i].id;
      }
    }

    sensor_msgs::PointCloud2 obj_msg;
    pcl::toROSMsg(label_cloud, obj_msg);
    obj_push_pub_.publish(obj_msg);
  }

  unsigned int sampleScore(std::vector<double>& scores)
  {
    SampleList samples;
    for (unsigned int i = 0; i < scores.size(); ++i)
    {
      if (scores[i] > 0.0)
      {
        PushSample p;
        p.id = i;
        p.weight = scores[i];
        samples.push_back(p);
      }
    }
    ROS_DEBUG_STREAM("Sampling from list of size: " << samples.size());

    std::sort(samples.begin(), samples.end(), PushSample::compareSamples);

    double running_cdf = 0.0;
    for (int i = samples.size() - 1; i >= 0; --i)
    {
      running_cdf += samples[i].weight;
      samples[i].cdf_weight = running_cdf;
    }
    for (unsigned int i = 0; i < samples.size(); ++i)
    {
      samples[i].cdf_weight /= running_cdf;
      samples[i].weight /= running_cdf;
    }

    float cdf_goal = randf();
    unsigned int idx = PushSample::cdfBinarySearch(samples, cdf_goal);
    return samples[idx].id;
  }

  //
  // I/O Methods
  //
  void displayBoundaryImage(cv::Mat& obj_img, Boundary& boundary,
                            std::string title, bool u8=true)
  {
    cv::Mat obj_disp_img(obj_img.size(), CV_32FC3);
    if (u8)
    {
      cv::Mat obj_img_f;
      obj_img.convertTo(obj_img_f, CV_32FC1, 30.0/255);
      cv::cvtColor(obj_img_f, obj_disp_img, CV_GRAY2BGR);
    }
    else
      cv::cvtColor(obj_img, obj_disp_img, CV_GRAY2BGR);
    cv::Vec3f green(0.0f, 1.0f, 0.0f);
    for (unsigned int i = 0; i < boundary.size(); ++i)
    {
      obj_disp_img.at<cv::Vec3f>(boundary[i].y, boundary[i].x) = green;
    }
    cv::imshow(title, obj_disp_img);
  }

  void draw3DBoundaries(std::vector<Boundary> boundaries, cv::Mat& obj_img,
                       int objs_size)
  {
    cv::Mat obj_disp_img(obj_img.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
    std::vector<cv::Vec3f> colors;
    for (int i = 0; i < objs_size; ++i)
    {
      cv::Vec3f rand_color;
      rand_color[0] = randf();
      rand_color[1] = randf();
      rand_color[2] = randf();
      colors.push_back(rand_color);
    }

    for (int r = 0; r < obj_img.rows; ++r)
    {
      for (int c = 0; c < obj_img.cols; ++c)
      {
        unsigned int id = obj_img.at<uchar>(r,c);
        if (id > 0)
        {
          obj_disp_img.at<cv::Vec3f>(r,c) = colors[id-1];
        }
      }
    }
    cv::Vec3f green(0.0, 1.0, 0.0);
    cv::Vec3f red(0.0, 0.0, 1.0);
    for (unsigned int b = 0; b < boundaries.size(); ++b)
    {
      cv::Vec3f color;
      if( boundaries[b].points3D.size() > 0)
        color = green;
      else
        color = red;
      for (unsigned int i = 0; i < boundaries[b].size(); ++i)
      {
        obj_disp_img.at<cv::Vec3f>(boundaries[b][i].y,
                                   boundaries[b][i].x) = color;
      }
    }
    cv::imshow("3D boundaries", obj_disp_img);
  }

  int getNextID()
  {
    return next_id_++;
  }
  //
  // Class member variables
  //
 protected:
  cv::Mat dx_kernel_;
  cv::Mat dy_kernel_;
  cv::Mat g_kernel_;
  FeatureTracker* ft_;
  PointCloudSegmentation* pcl_segmenter_;
  XYZPointCloud prev_cloud_down_;
  XYZPointCloud prev_objs_down_;
  ProtoObjects prev_proto_objs_;
  ProtoObjects cur_proto_objs_;
  int callback_count_;
  int next_id_;

 public:
  double min_pushing_x_;
  double max_pushing_x_;
  double min_pushing_y_;
  double max_pushing_y_;
  std::string workspace_frame_;
  ros::Publisher obj_push_pub_;
  bool use_weighted_edges_;
  bool threshold_edges_;
  double depth_edge_weight_;
  double edge_weight_thresh_;
  double depth_edge_weight_thresh_;
  double max_push_angle_;
  double min_push_angle_;
  double boundary_ransac_thresh_;
  int min_edge_length_;
};

class ObjectSingulationNode
{
 public:
  ObjectSingulationNode(ros::NodeHandle &n) :
      n_(n), n_private_("~"),
      image_sub_(n, "color_image_topic", 1),
      depth_sub_(n, "depth_image_topic", 1),
      cloud_sub_(n, "point_cloud_topic", 1),
      sync_(MySyncPolicy(15), image_sub_, depth_sub_, cloud_sub_),
      it_(n), tf_(), ft_("pushing_perception"),
      pcl_segmenter_(&ft_, &tf_),
      os_(&ft_, &pcl_segmenter_),
      have_depth_data_(false), tracking_(false),
      tracker_initialized_(false), tracker_count_(0)
  {
    // Get parameters from the server
    n_private_.param("crop_min_x", crop_min_x_, 0);
    n_private_.param("crop_max_x", crop_max_x_, 640);
    n_private_.param("crop_min_y", crop_min_y_, 0);
    n_private_.param("crop_max_y", crop_max_y_, 480);
    n_private_.param("display_wait_ms", display_wait_ms_, 3);
    n_private_.param("min_workspace_x", min_workspace_x_, 0.0);
    n_private_.param("min_workspace_y", min_workspace_y_, 0.0);
    n_private_.param("min_workspace_z", min_workspace_z_, 0.0);
    n_private_.param("max_workspace_x", max_workspace_x_, 0.0);
    n_private_.param("max_workspace_y", max_workspace_y_, 0.0);
    n_private_.param("max_workspace_z", max_workspace_z_, 0.0);
    n_private_.param("min_pushing_x", os_.min_pushing_x_, 0.0);
    n_private_.param("min_pushing_y", os_.min_pushing_y_, 0.0);
    n_private_.param("max_pushing_x", os_.max_pushing_x_, 0.0);
    n_private_.param("max_pushing_y", os_.max_pushing_y_, 0.0);
    std::string default_workspace_frame = "/torso_lift_link";
    n_private_.param("workspace_frame", workspace_frame_,
                     default_workspace_frame);
    os_.workspace_frame_ = workspace_frame_;

    n_private_.param("use_weighted_edges", os_.use_weighted_edges_, false);
    n_private_.param("threshold_edges", os_.threshold_edges_, false);
    n_private_.param("edge_weight_thresh", os_.edge_weight_thresh_, 0.5);
    n_private_.param("depth_edge_weight_thresh", os_.depth_edge_weight_thresh_,
                     0.5);
    n_private_.param("depth_edge_weight", os_.depth_edge_weight_, 0.75);
    n_private_.param("max_pushing_angle", os_.max_push_angle_, M_PI*0.5);
    n_private_.param("min_pushing_angle", os_.min_push_angle_, -M_PI*0.5);
    n_private_.param("boundary_ransac_thresh", os_.boundary_ransac_thresh_,
                     0.01);
    n_private_.param("min_edge_length", os_.min_edge_length_, 3);

    n_private_.param("min_table_z", pcl_segmenter_.min_table_z_, -0.5);
    n_private_.param("max_table_z", pcl_segmenter_.max_table_z_, 1.5);
    pcl_segmenter_.min_workspace_x_ = min_workspace_x_;
    pcl_segmenter_.max_workspace_x_ = max_workspace_x_;
    pcl_segmenter_.min_workspace_z_ = min_workspace_z_;
    pcl_segmenter_.max_workspace_z_ = max_workspace_z_;

    n_private_.param("autostart_tracking", tracking_, false);
    n_private_.param("autostart_pcl_segmentation", autorun_pcl_segmentation_,
                     false);
    n_private_.param("use_guided_pushes", use_guided_pushes_, true);

    n_private_.param("num_downsamples", num_downsamples_, 2);
    pcl_segmenter_.num_downsamples_ = num_downsamples_;
    std::string cam_info_topic_def = "/kinect_head/rgb/camera_info";
    n_private_.param("cam_info_topic", cam_info_topic_,
                     cam_info_topic_def);
    n_private_.param("table_ransac_thresh", pcl_segmenter_.table_ransac_thresh_,
                     0.01);
    n_private_.param("table_ransac_angle_thresh",
                     pcl_segmenter_.table_ransac_angle_thresh_, 30.0);

    n_private_.param("surf_hessian_thresh", ft_.surf_.hessianThreshold,
                     150.0);
    bool use_fast;
    n_private_.param("use_fast_corners", use_fast, false);
    ft_.setUseFast(use_fast);
    n_private_.param("pcl_cluster_tolerance", pcl_segmenter_.cluster_tolerance_,
                     0.25);
    n_private_.param("pcl_difference_thresh", pcl_segmenter_.cloud_diff_thresh_,
                     0.01);
    n_private_.param("pcl_min_cluster_size", pcl_segmenter_.min_cluster_size_,
                     100);
    n_private_.param("pcl_max_cluster_size", pcl_segmenter_.max_cluster_size_,
                     2500);
    n_private_.param("pcl_voxel_downsample_res", pcl_segmenter_.voxel_down_res_,
                     0.005);
    n_private_.param("pcl_cloud_intersect_thresh",
                     pcl_segmenter_.cloud_intersect_thresh_, 0.005);
    n_private_.param("pcl_concave_hull_alpha", pcl_segmenter_.hull_alpha_,
                     0.1);
    n_private_.param("use_pcl_voxel_downsample", pcl_segmenter_.use_voxel_down_,
                     true);
    n_private_.param("default_push_dist", default_push_dist_, 0.1);

    // Setup ros node connections
    sync_.registerCallback(&ObjectSingulationNode::sensorCallback,
                           this);
    push_pose_server_ = n_.advertiseService(
        "get_push_pose", &ObjectSingulationNode::getPushPose, this);
    table_location_server_ = n_.advertiseService(
        "get_table_location", &ObjectSingulationNode::getTableLocation,
        this);
    pcl_segmenter_.pcl_obj_seg_pub_ = n_.advertise<sensor_msgs::PointCloud2>(
        "separate_table_objs", 1000);
    pcl_segmenter_.pcl_down_pub_ = n_.advertise<sensor_msgs::PointCloud2>(
        "downsampled_objs", 1000);
    os_.obj_push_pub_ = n_.advertise<sensor_msgs::PointCloud2>(
        "object_singulation_cloud", 1000);
  }

  void sensorCallback(const sensor_msgs::ImageConstPtr& img_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg,
                      const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    if (!camera_initialized_)
    {
      cam_info_ = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
          cam_info_topic_, n_, ros::Duration(5.0));
      camera_initialized_ = true;
      pcl_segmenter_.cam_info_ = cam_info_;
    }
    // Convert images to OpenCV format
    cv::Mat color_frame(bridge_.imgMsgToCv(img_msg));
    cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));

    // Swap kinect color channel order
    cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);

    // Transform point cloud into the correct frame and convert to PCL struct
    XYZPointCloud cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    tf_.waitForTransform(workspace_frame_, cloud.header.frame_id,
                         cloud.header.stamp, ros::Duration(0.5));
    pcl_ros::transformPointCloud(workspace_frame_, cloud, cloud, tf_);

    // Convert nans to zeros
    for (int r = 0; r < depth_frame.rows; ++r)
    {
      float* depth_row = depth_frame.ptr<float>(r);
      for (int c = 0; c < depth_frame.cols; ++c)
      {
        float cur_d = depth_row[c];
        if (isnan(cur_d))
        {
          depth_row[c] = 0.0;
        }
      }
    }

    cv::Mat workspace_mask(color_frame.rows, color_frame.cols, CV_8UC1,
                           cv::Scalar(255));
    // Black out pixels in color and depth images outside of workspace
    // As well as outside of the crop window
    for (int r = 0; r < color_frame.rows; ++r)
    {
      uchar* workspace_row = workspace_mask.ptr<uchar>(r);
      for (int c = 0; c < color_frame.cols; ++c)
      {
        // NOTE: Cloud is accessed by at(column, row)
        pcl::PointXYZ cur_pt = cloud.at(c, r);
        if (cur_pt.x < min_workspace_x_ || cur_pt.x > max_workspace_x_ ||
            cur_pt.y < min_workspace_y_ || cur_pt.y > max_workspace_y_ ||
            cur_pt.z < min_workspace_z_ || cur_pt.z > max_workspace_z_ ||
            r < crop_min_y_ || c < crop_min_x_ || r > crop_max_y_ ||
            c > crop_max_x_ )
        {
          workspace_row[c] = 0;
        }
      }
    }

    // Downsample everything first
    cv::Mat color_frame_down = downSample(color_frame, num_downsamples_);
    cv::Mat depth_frame_down = downSample(depth_frame, num_downsamples_);
    cv::Mat workspace_mask_down = downSample(workspace_mask, num_downsamples_);

    // Save internally for use in the service callback
    prev_color_frame_ = cur_color_frame_.clone();
    prev_depth_frame_ = cur_depth_frame_.clone();
    prev_workspace_mask_ = cur_workspace_mask_.clone();
    prev_camera_header_ = cur_camera_header_;

    // Update the current versions
    cur_color_frame_ = color_frame_down.clone();
    cur_depth_frame_ = depth_frame_down.clone();
    cur_workspace_mask_ = workspace_mask_down.clone();
    cur_point_cloud_ = cloud;
    have_depth_data_ = true;
    cur_camera_header_ = img_msg->header;
    pcl_segmenter_.cur_camera_header_ = cur_camera_header_;

    // Debug stuff
    if (autorun_pcl_segmentation_) getPushPose(default_push_dist_,
                                               use_guided_pushes_);

    // Display junk
#ifdef DISPLAY_INPUT_COLOR
    cv::imshow("color", cur_color_frame_);
#endif // DISPLAY_INPUT_COLOR
#ifdef DISPLAY_INPUT_DEPTH
    double depth_max = 1.0;
    cv::minMaxLoc(cur_depth_frame_, NULL, &depth_max);
    cv::Mat depth_display = cur_depth_frame_.clone();
    depth_display /= depth_max;
    cv::imshow("input_depth", depth_display);
#endif // DISPLAY_INPUT_DEPTH
#ifdef DISPLAY_WORKSPACE_MASK
    cv::imshow("workspace_mask", cur_workspace_mask_);
#endif // DISPLAY_WORKSPACE_MASK
#ifdef DISPLAY_WAIT
    cv::waitKey(display_wait_ms_);
#endif // DISPLAY_WAIT
  }

  /**
   * Service request callback method to return a location and orientation for
   * the robot to push.
   *
   * @param req The service request
   * @param res The service response
   *
   * @return true if successfull, false otherwise
   */
  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    if ( have_depth_data_ )
    {
      res = getPushPose(req.push_dist, req.use_guided);
    }
    else
    {
      ROS_ERROR_STREAM("Calling getPushPose prior to receiving sensor data.");
      return false;
    }
    return true;
  }

  /**
   * Wrapper method to call the push pose from the ObjectSingulation class
   *
   * @param use_guided find a random pose if false, otherwise calculate using
   *                   the ObjectSingulation method
   *
   * @return The PushPose
   */
  PushVector getPushPose(double push_dist=0.1, bool use_guided=true)
  {
    if (!use_guided)
    {
      return os_.findRandomPushPose(cur_point_cloud_);
    }
    else
    {
      return os_.getPushVector(push_dist,
                               cur_color_frame_, cur_depth_frame_,
                               cur_point_cloud_, cur_workspace_mask_);
    }
  }

  /**
   * ROS Service callback method for determining the location of a table in the
   * scene
   *
   * @param req The service request
   * @param res The service response
   *
   * @return true if successfull, false otherwise
   */
  bool getTableLocation(LocateTable::Request& req, LocateTable::Response& res)
  {
    if ( have_depth_data_ )
    {
      res.table_centroid = getTablePlane(cur_point_cloud_);
      if ((res.table_centroid.pose.position.x == 0.0 &&
           res.table_centroid.pose.position.y == 0.0 &&
           res.table_centroid.pose.position.z == 0.0) ||
          res.table_centroid.pose.position.x < 0.0)
      {
        ROS_ERROR_STREAM("No plane found, leaving");
        res.found_table = false;
        return false;
      }
      res.found_table = true;
    }
    else
    {
      ROS_ERROR_STREAM("Calling getTableLocation prior to receiving sensor data.");
      res.found_table = false;
      return false;
    }
    return true;
  }

  /**
   * Calculate the location of the dominant plane (table) in a point cloud
   *
   * @param cloud The point cloud containing a table
   *
   * @return The estimated 3D centroid of the table
   */
  PoseStamped getTablePlane(XYZPointCloud& cloud)
  {
    XYZPointCloud obj_cloud, table_cloud;
    // TODO: Comptue the hull on the first call
    Eigen::Vector4f table_centroid = pcl_segmenter_.getTablePlane(cloud,
                                                                  obj_cloud,
                                                                  table_cloud/*,
                                                                  true*/);
    PoseStamped p;
    p.pose.position.x = table_centroid[0];
    p.pose.position.y = table_centroid[1];
    p.pose.position.z = table_centroid[2];
    p.header = cloud.header;
    ROS_INFO_STREAM("Table centroid is: ("
                    << p.pose.position.x << ", "
                    << p.pose.position.y << ", "
                    << p.pose.position.z << ")");
    return p;
  }

  //
  // Helper Methods
  //
  cv::Mat downSample(cv::Mat data_in, int scales)
  {
    cv::Mat out = data_in.clone();
    for (int i = 0; i < scales; ++i)
    {
      cv::pyrDown(data_in, out);
      data_in = out;
    }
    return out;
  }

  cv::Mat upSample(cv::Mat data_in, int scales)
  {
    cv::Mat out = data_in.clone();
    for (int i = 0; i < scales; ++i)
    {
      // NOTE: Currently assumes even cols, rows for data_in
      cv::Size out_size(data_in.cols*2, data_in.rows*2);
      cv::pyrUp(data_in, out, out_size);
      data_in = out;
    }
    return out;
  }

  XYZPointCloud getMaskedPointCloud(XYZPointCloud& input_cloud, cv::Mat& mask_in)
  {
    // TODO: Assert that input_cloud is shaped
    cv::Mat mask = upSample(mask_in, num_downsamples_);

    // Select points from point cloud that are in the mask:
    pcl::PointIndices mask_indices;
    mask_indices.header = input_cloud.header;
    for (int y = 0; y < mask.rows; ++y)
    {
      uchar* mask_row = mask.ptr<uchar>(y);
      for (int x = 0; x < mask.cols; ++x)
      {
        if (mask_row[x] != 0)
        {
          mask_indices.indices.push_back(y*input_cloud.width + x);
        }
      }
    }

    XYZPointCloud masked_cloud;
    pcl::copyPointCloud(input_cloud, mask_indices, masked_cloud);
    return masked_cloud;
  }

  /**
   * Executive control function for launching the node.
   */
  void spin()
  {
    while(n_.ok())
    {
      ros::spinOnce();
    }
  }

 protected:
  ros::NodeHandle n_;
  ros::NodeHandle n_private_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Synchronizer<MySyncPolicy> sync_;
  image_transport::ImageTransport it_;
  sensor_msgs::CameraInfo cam_info_;
  sensor_msgs::CvBridge bridge_;
  tf::TransformListener tf_;
  ros::ServiceServer push_pose_server_;
  ros::ServiceServer table_location_server_;
  cv::Mat cur_color_frame_;
  cv::Mat cur_depth_frame_;
  cv::Mat cur_workspace_mask_;
  cv::Mat prev_color_frame_;
  cv::Mat prev_depth_frame_;
  cv::Mat prev_workspace_mask_;
  std_msgs::Header cur_camera_header_;
  std_msgs::Header prev_camera_header_;
  XYZPointCloud cur_point_cloud_;
  FeatureTracker ft_;
  PointCloudSegmentation pcl_segmenter_;
  ObjectSingulation os_;
  bool have_depth_data_;
  int crop_min_x_;
  int crop_max_x_;
  int crop_min_y_;
  int crop_max_y_;
  int display_wait_ms_;
  double min_workspace_x_;
  double max_workspace_x_;
  double min_workspace_y_;
  double max_workspace_y_;
  double min_workspace_z_;
  double max_workspace_z_;
  int num_downsamples_;
  std::string workspace_frame_;
  PoseStamped table_centroid_;
  bool tracking_;
  bool tracker_initialized_;
  bool camera_initialized_;
  std::string cam_info_topic_;
  int tracker_count_;
  bool autorun_pcl_segmentation_;
  bool use_guided_pushes_;
  double default_push_dist_;
};

int main(int argc, char ** argv)
{
  srand(time(NULL));
  ros::init(argc, argv, "object_singulation_node");
  ros::NodeHandle n;
  ObjectSingulationNode singulation_node(n);
  singulation_node.spin();
  return 0;
}
