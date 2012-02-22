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
#include <boost/shared_ptr.hpp>

// tabletop_pushing
#include <tabletop_pushing/PushPose.h>
#include <tabletop_pushing/LocateTable.h>

// STL
#include <vector>
#include <deque>
#include <queue>
#include <set>
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
// #define DISPLAY_INPUT_DEPTH 1
// #define DISPLAY_WORKSPACE_MASK 1
#define DISPLAY_PROJECTED_OBJECTS 1
// #define DISPLAY_LINKED_EDGES 1
 #define DISPLAY_CHOSEN_BOUNDARY 1
// #define DISPLAY_CLOUD_DIFF 1
#define DISPLAY_3D_BOUNDARIES 1
#define DISPLAY_PUSH_VECTOR 1
#define DISPLAY_WAIT 1
#define DEBUG_PUSH_HISTORY 1

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
using boost::shared_ptr;

class Boundary : public std::vector<cv::Point>
{
 public:
  std::vector<pcl::PointXYZ> points3D;
  int object_id;
  double ort;
  bool external;
  bool too_short;
};

class ProtoTabletopObject
{
 public:
  XYZPointCloud cloud;
  Eigen::Vector4f centroid;
  int id;
  bool moved;
  Eigen::Matrix4f transform;
  std::vector<int> boundary_angle_dist;
  std::vector<int> push_history;
};

typedef std::deque<ProtoTabletopObject> ProtoObjects;

class PointCloudSegmentation
{
 public:
  PointCloudSegmentation(shared_ptr<tf::TransformListener> tf) :
      tf_(tf)
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
      po.push_history.clear();
      po.boundary_angle_dist.clear();
      pcl::copyPointCloud(objects_cloud, clusters[i], po.cloud);
      pcl::compute3DCentroid(po.cloud, po.centroid);
      po.id = i;
      po.moved = false;
      po.transform = Eigen::Matrix4f::Identity();
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
  double ICPProtoObjects(ProtoTabletopObject& a, ProtoTabletopObject& b,
                         Eigen::Matrix4f& transform)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
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
      const pcl::PointXYZ pt0 = cloud0.at(i);
      for (unsigned int j = 0; j < cloud1.size(); ++j)
      {
        const pcl::PointXYZ pt1 = cloud1.at(j);
        if (dist(pt0, pt1) < cloud_intersect_thresh_) return true;
      }
    }
    return false;
  }

  float pointLineXYDist(pcl::PointXYZ p,Eigen::Vector3f vec,Eigen::Vector4f base)
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

  XYZPointCloud lineCloudIntersection(XYZPointCloud& cloud, Eigen::Vector3f vec,
                                        Eigen::Vector4f base)
  {
    // TODO: Define parametric model of the line defined by base and vec and
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
  cv::Mat displayObjectImage(cv::Mat& obj_img,
                             std::string win_name="projected objects",
                             bool use_display=true)
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

  void projectPointCloudIntoImage(XYZPointCloud& cloud, cv::Mat& lbl_img,
                                  std::string target_frame, unsigned int id=1)
  {
    for (unsigned int i = 0; i < cloud.size(); ++i)
    {
      cv::Point img_idx = projectPointIntoImage(cloud.at(i),
                                                cloud.header.frame_id,
                                                target_frame);
      lbl_img.at<uchar>(img_idx.y, img_idx.x) = id;
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
      ROS_ERROR_STREAM("Fucked.");
      ROS_ERROR_STREAM(e.what());
    }
    return img_loc;
  }

  Eigen::Vector4f getTableCentroid()
  {
    return table_centroid_;
  }

 protected:
  shared_ptr<tf::TransformListener> tf_;
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
          Eigen::Vector3f _push_vec, unsigned int _obj_id,
          unsigned int _split_id, double _push_dist=0.1) :
      obj(_obj), push_angle(_push_angle), push_unit_vec(_push_vec),
      object_id(_obj_id), split_id(_split_id), push_dist(_push_dist)
  {
  }
  ProtoTabletopObject obj;
  double push_angle;
  Eigen::Vector3f push_unit_vec;
  unsigned int object_id;
  unsigned int split_id;
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

  Eigen::Vector4f getMovedPoint(geometry_msgs::Point p)
  {
    Eigen::Vector4f moved;
    moved[0] = p.x + push_unit_vec[0]*push_dist;
    moved[1] = p.y + push_unit_vec[1]*push_dist;
    moved[2] = p.z + push_unit_vec[2]*push_dist;
    moved[3] = 1.0f;
    return moved;
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
    if (use_displays)
    {
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
    }
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
  ObjectSingulation(shared_ptr<PointCloudSegmentation> pcl_segmenter) :
      pcl_segmenter_(pcl_segmenter), callback_count_(0), next_id_(0),
      initialized_(false)
  {
    // Create derivative kernels for edge calculation
    cv::getDerivKernels(dy_kernel_, dx_kernel_, 1, 0, CV_SCHARR, true, CV_32F);
    cv::flip(dy_kernel_, dy_kernel_, -1);
    cv::transpose(dy_kernel_, dx_kernel_);
  }

  bool initialize(cv::Mat& color_img, cv::Mat& depth_img,
                  XYZPointCloud& cloud, cv::Mat& workspace_mask)
  {
    callback_count_ = 0;
    ProtoObjects objs = calcProtoObjects(cloud);
    initialized_ = true;
    callback_count_++;
    return true;
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
  PushVector getPushVector(double push_dist, bool no_push_calc,
                           cv::Mat& color_img,
                           cv::Mat& depth_img, XYZPointCloud& cloud,
                           cv::Mat& workspace_mask)
  {
    if (!initialized_)
    {
      ROS_WARN_STREAM("Trying to get a push vector before initializing.");
      PushVector push_pose;
      push_pose.object_id = 0;
      push_pose.no_push = true;
      push_pose.num_objects = 0;
      return push_pose;
    }
    // Move current proto objects to prev proto objects
    prev_proto_objs_.clear();
    for (unsigned int i = 0; i < cur_proto_objs_.size(); ++i)
    {
      prev_proto_objs_.push_back(cur_proto_objs_[i]);
    }
    ProtoObjects objs = calcProtoObjects(cloud);

    if (no_push_calc)
    {
      PushVector push_vector;
      push_vector.no_push = true;
      push_vector.num_objects = objs.size();
      ++callback_count_;
      return push_vector;
    }

    cv::Mat boundary_img;
    std::vector<Boundary> boundaries = getObjectBoundaryStrengths(
        color_img, depth_img, workspace_mask, boundary_img);
    PushVector push_vector = determinePushPose(push_dist, boundary_img,
                                               cur_proto_objs_,
                                               boundaries, cloud);
    push_vector.num_objects = cur_proto_objs_.size();
    if (push_vector.object_id == cur_proto_objs_.size())
    {
      ROS_WARN_STREAM("No push vector selected.");
      push_vector.no_push = true;
      ++callback_count_;
      return push_vector;
    }

#ifdef DEBUG_PUSH_HISTORY
    //drawObjectHists(obj_lbl_img, cur_proto_objs_);
    drawObjectHists(color_img, cur_proto_objs_);
#endif // DEBUG_PUSH_HISTORY
#ifdef DISPLAY_3D_BOUNDARIES
    if (use_displays_ || write_to_disk_)
    {
      draw3DBoundaries(boundaries, color_img, objs.size());
    }
#endif // DISPLAY_3D_BOUNDARIES
    ++callback_count_;
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
      p.no_push = true;
      return p;
    }
    ROS_INFO_STREAM("Found " << pushable_obj_idx.size()
                    << " pushable proto objects");
    int rand_idx = pushable_obj_idx[rand() % pushable_obj_idx.size()];
    Eigen::Vector4f obj_xyz_centroid = objs[rand_idx].centroid;
    double rand_orientation = (randf()*
                               (max_push_angle_- min_push_angle_) +
                               min_push_angle_);
    p.start_point.x = obj_xyz_centroid[0];
    p.start_point.y = obj_xyz_centroid[1];
    // Set z to be the table height
    p.start_point.z = obj_xyz_centroid[2];
    // Choose a random orientation
    p.push_angle = rand_orientation;

    sensor_msgs::PointCloud2 obj_push_msg;
    pcl::toROSMsg(objs[rand_idx].cloud, obj_push_msg);
    obj_push_pub_.publish(obj_push_msg);

    ROS_INFO_STREAM("Chosen push pose is at: (" << p.start_point.x << ", "
                    << p.start_point.y << ", " << p.start_point.z
                    << ") with orientation of: " << p.push_angle);
    p.num_objects = pushable_obj_idx.size();
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
    XYZPointCloud objs_cloud;
    ProtoObjects objs = pcl_segmenter_->findTabletopObjects(cloud, objs_cloud,
                                                            true);
    ROS_INFO_STREAM("Found " << objs.size() << " objects!");
    for (unsigned int i = 0; i < objs.size(); ++i)
    {
      objs[i].push_history.resize(num_angle_bins_, 0);
    }
    XYZPointCloud cur_objs_down = pcl_segmenter_->downsampleCloud(objs_cloud,
                                                                  true);
    ProtoObjects cur_objs;
    if (callback_count_ > 0)
    {
      // Determine where stuff has moved
      ProtoObjects moved_regions = pcl_segmenter_->getMovedRegions(
          prev_objs_down_, cur_objs_down);
      // Match these moved regions to the previous objects
      pcl_segmenter_->matchMovedRegions(prev_proto_objs_, moved_regions);
      // Match the moved objects to their new locations
      updateMovedObjs(objs, prev_proto_objs_);
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
    cur_proto_objs_.clear();
    cur_proto_objs_ = objs;
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
      ROS_DEBUG_STREAM("Objects merged from " << prev_objs.size() << " to " <<
                      cur_objs.size());
      // TODO: Something different for merging, check which moved objects
      // combined with which unmoved objects
    }
    else if (split)
    {
      ROS_INFO_STREAM("Objects split from " << prev_objs.size() << " to " <<
                      cur_objs.size());
      int num_moved = 0;
      int num_unmoved = 0;
      for (unsigned int i = 0; i < prev_objs.size(); ++i)
      {
        if (prev_objs[i].moved)
        {
          num_moved++;
        }
        else
        {
          num_unmoved++;
        }
      }
      ROS_INFO_STREAM("num_moved: " << num_moved);
      ROS_INFO_STREAM("num_moved: " << num_unmoved);
    }
    else
    {
      ROS_DEBUG_STREAM("Same number of objects: " << prev_objs.size());
    }

    std::vector<bool> matched = matchUnmoved(cur_objs, prev_objs);
    matchMoved(cur_objs, prev_objs, matched, split);
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
            cur_objs[min_idx].push_history = prev_objs[i].push_history;
            cur_objs[min_idx].transform = prev_objs[i].transform;
          }

          matched[min_idx] = true;
        }
      }
    }
    return matched;
  }

  void matchMoved(ProtoObjects& cur_objs, ProtoObjects& prev_objs,
                  std::vector<bool> matched, bool split)
  {
    for (unsigned int i = 0; i < prev_objs.size(); ++i)
    {
      if (prev_objs[i].moved)
      {
        double min_score = std::numeric_limits<double>::max();
        unsigned int min_idx = cur_objs.size();
        // Match the moved objects to their new locations
        ROS_DEBUG_STREAM("Finding match for object : " << prev_objs[i].id);
        Eigen::Matrix4f min_transform;
        for (unsigned int j = 0; j < cur_objs.size(); ++j)
        {
          // TODO: Deal with split here
          if (!matched[j])
          {
            // Run ICP to match between frames
            Eigen::Matrix4f transform;
            ROS_INFO_STREAM("ICP of " << i << " to " << j);
            double cur_score = pcl_segmenter_->ICPProtoObjects(prev_objs[i],
                                                               cur_objs[j],
                                                               transform);
            if (cur_score < min_score)
            {
              min_score = cur_score;
              min_idx = j;
              min_transform = transform;
            }
          }
        }
        if (min_idx < cur_objs.size())
        {
          // TODO: If score is too bad ignore / instantiate new
          ROS_INFO_STREAM("Prev moved obj: " << prev_objs[i].id  << ", " << i
                          << " maps to cur " << min_idx << " : " << min_score);
          if (matched[min_idx])
          {
          }
          else
          {
            cur_objs[min_idx].id = prev_objs[i].id;
            cur_objs[min_idx].push_history = prev_objs[i].push_history;
            cur_objs[min_idx].transform = min_transform*prev_objs[i].transform;
            matched[min_idx] = true;
          }
        }
        else
        {
          ROS_WARN_STREAM("No match for moved previus object: "
                          << prev_objs[i].id);
        }
      }
    }
    for (unsigned int i = 0; i < matched.size(); ++i)
    {
      if (!matched[i]) cur_objs[i].id = getNextID();
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
                               std::vector<Boundary>& boundaries,
                               XYZPointCloud cloud)
  {
    ROS_INFO_STREAM("determinePushPose()");
    cv::Mat obj_lbl_img = pcl_segmenter_->projectProtoObjectsIntoImage(
        objs, boundary_img.size(), workspace_frame_);

#ifdef DISPLAY_PROJECTED_OBJECTS
    if (use_displays_ || write_to_disk_)
    {
      drawObjectTransformAxises(obj_lbl_img, objs);
    }
#endif // DISPLAY_PROJECTED_OBJECTS

    get3DBoundaries(boundaries, cloud);
    associate3DBoundaries(boundaries, objs, obj_lbl_img);

    std::vector<unsigned int> singulated = checkSingulated(boundaries, objs);
    if (singulated.size() > 0)
    {
      std::stringstream sing_stream;
      for (unsigned int i = 0; i < singulated.size(); ++i)
      {
        sing_stream << i << " ";
      }
      ROS_INFO_STREAM("The following objects are singulated: " <<
                      sing_stream.str());
    }
    if (singulated.size() == objs.size())
    {
      ROS_WARN_STREAM("Boundary has no ID!");
      PushVector push_pose;
      push_pose.object_id = objs.size();
      push_pose.no_push = true;
      push_pose.singulated = singulated;
      return push_pose;
    }
    Boundary test_boundary = chooseTestBoundary(boundaries, objs);
    PushVector push = determinePushVector(push_dist, test_boundary, objs,
                                          obj_lbl_img, cloud);
    // Increment the push direction of the object to be pushed
    const int push_idx = quantizeAngle(test_boundary.ort);
    cur_proto_objs_[push.object_id].push_history[push_idx]++;

    ROS_INFO_STREAM("Chose to push object: " << push.object_id);
    ROS_INFO_STREAM("Push angle is : " << push.push_angle);
    ROS_INFO_STREAM("Push angle is in bin : " << push_idx);
    push.singulated = singulated;
    return push;
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
                                 XYZPointCloud& cloud)
  {
    ROS_INFO_STREAM("determinePushVector()");

    // Split point cloud at location of the boundary
    ProtoObjects split_objs3D = splitObject3D(boundary, objs[boundary.
                                                             object_id]);
    // Need to transform this into the world frame
    float push_angle = getWorldFrameAngleFromObjFrame(
        boundary.ort, objs[boundary.object_id].transform);
    PushVector push_pose = determinePushDirection(push_dist, push_angle,
                                                  split_objs3D[0],
                                                  split_objs3D[1], objs,
                                                  boundary.object_id,
                                                  obj_lbl_img, boundary);
    push_pose.header.frame_id = workspace_frame_;
    push_pose.object_id = boundary.object_id;
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
      combined_edges = cv::max(edge_img_masked, depth_edge_img_masked);
    }

    // Link edges into object boundary hypotheses
    std::vector<Boundary> boundaries = LinkEdges::edgeLink(combined_edges,
                                                           min_edge_length_,
                                                           use_displays_);
    return boundaries;
  }

  void get3DBoundaries(std::vector<Boundary>& boundaries, XYZPointCloud& cloud)
  {
    ROS_INFO_STREAM("get3DBoundaries()");
    for (unsigned int b = 0; b < boundaries.size(); ++b)
    {
      get3DBoundary(boundaries[b], cloud);
    }
  }

  void get3DBoundary(Boundary& b, XYZPointCloud& cloud)
  {
    b.points3D.clear();
    for (unsigned int i = 0; i < b.size(); ++i)
    {
      // NOTE: I don't think this is what it should be, lets try though...
      // NOTE: need to upsample the indices here
      pcl::PointXYZ p = cloud.at(b[i].x*upscale_, b[i].y*upscale_);
      // Don't add empty points
      if ((p.x == 0.0 && p.y == 0.0 && p.z == 0.0 ) || isnan(p.x) ||
          isnan(p.y) || isnan(p.z)) continue;
      b.points3D.push_back(p);
    }
  }

  void associate3DBoundaries(std::vector<Boundary>& boundaries,
                             ProtoObjects& objs, cv::Mat& obj_lbl_img)
  {
    ROS_INFO_STREAM("associate3DBoundaries()");
    // Clear boundary_angle_dist for all objects
    for (unsigned int o = 0; o < objs.size(); ++o)
    {
      objs[o].boundary_angle_dist.clear();
      objs[o].boundary_angle_dist.resize(num_angle_bins_, 0);
    }
    int no_overlap_count = 0;
    for (unsigned int b = 0; b < boundaries.size(); ++b)
    {
      // NOTE: default to no match, only update if all criterian are met
      boundaries[b].object_id = objs.size();
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
        no_overlap_count++;
      }
      else
      {
        boundaries[b].object_id = max_id;
        // Don't add short boundaries
        if (boundaries[b].points3D.size() >= min_boundary_length_)
        {
          boundaries[b].too_short = false;
          ProtoObjects pos = splitObject3D(boundaries[b], objs[max_id]);
          const unsigned int s0 = pos[0].cloud.size();
          const unsigned int s1 = pos[1].cloud.size();

          // NOTE: Don't add external object boundaries
          if (s0 > min_cluster_size_ && s1 > min_cluster_size_)
          {
            boundaries[b].ort = getObjFrameBoundaryOrientation(
                boundaries[b], objs[max_id].transform);
            int angle_idx = quantizeAngle(boundaries[b].ort);
            objs[max_id].boundary_angle_dist[angle_idx]++;
            boundaries[b].external = false;
          }
          else
          {
            boundaries[b].external = true;
          }
        }
        else
        {
          boundaries[b].too_short = true;
          boundaries[b].external = false;
        }
      }
    }
  }

  int getObjFrameAngleFromWorldFrame(float theta, Eigen::Matrix4f& t)
  {
    // Get vector from push_angle [cos(theta), sin(theta), 0, 1]
    Eigen::Vector4f x(std::cos(theta), std::sin(theta), 0.0f, 1.0f);
    // apply transform;
    Eigen::Vector4f y = t.inverse()*x;
    // Extract angle
    float angle = std::atan2(y[1], y[0]);
    return angle;
  }

  float getWorldFrameAngleFromObjFrame(float theta, Eigen::Matrix4f& t)
  {
    Eigen::Vector4f x(std::cos(theta), std::sin(theta), 0.0f, 1.0f);
    // apply transform;
    Eigen::Vector4f y = t*x;
    // Extract angle
    float angle = std::atan2(y[1], y[0]);
    return angle;
  }

  int getObjFrameIndexFromWorldAngle(float theta_world, Eigen::Matrix4f& t)
  {
    float angle = getObjFrameAngleFromWorldFrame(theta_world, t);
    return quantizeAngle(angle);
  }

  int getObjFrameBoundaryOrientationIndex(Boundary& b, Eigen::Matrix4f& t)
  {
    float angle = getObjFrameBoundaryOrientation(b, t);
    return quantizeAngle(angle);
  }

  float getObjFrameBoundaryOrientation(Boundary& b, Eigen::Matrix4f& t)
  {
    Eigen::Vector3f b_vect3 = getRANSACXYVector(b);

    // Rotate based on object transform history
    Eigen::Vector4f b_vect(b_vect3[0], b_vect3[1], b_vect3[2], 1.0f);
    Eigen::Vector4f b_vect_obj = t.inverse()*b_vect;

    float angle = std::atan2(b_vect_obj[1], b_vect_obj[0]);

    return subPiAngle(angle);
  }

  int quantizeAngle(float angle)
  {
    // TODO: Make into an assert
    if (angle < -M_PI/2.0 || angle > M_PI/2.0)
    {
      ROS_WARN_STREAM("Quantizing angle: " << angle << " outside of range");
      angle = subPiAngle(angle);
      ROS_WARN_STREAM("Converted to: " << angle);
    }
    int bin = static_cast<int>(((angle + M_PI/2.0)/M_PI)*num_angle_bins_);
    return std::max(std::min(bin, num_angle_bins_-1), 0);
  }

  float subPiAngle(float angle)
  {
    // NOTE: All angles should be between -pi/2 and pi/2 (only want gradient)
    while ( angle < -M_PI/2 )
    {
      angle += M_PI;
    }
    while ( angle > M_PI/2 )
    {
      angle -= M_PI;
    }
    return angle;
  }

  Eigen::Vector3f getRANSACXYVector(Boundary& b)
  {
    Eigen::Vector3f l_pt;
    return getRANSACXYVector(b, l_pt);
  }

  Eigen::Vector3f getRANSACXYVector(Boundary& b, Eigen::Vector3f& l_pt,
                                    bool debug_fitting=false)
  {
    XYZPointCloud cloud;
    cloud.resize(b.points3D.size());
    for (unsigned int i = 0; i < b.points3D.size(); ++i)
    {
      cloud.at(i) = b.points3D[i];
      // NOTE: This is kind of a hack, instead of setting to 0.0, I set to the z
      // of the first point to try and get around numerical approximations
      // NOTE: Helps with visualization if the line is at the approximate height
      // in the world
      cloud.at(i).z = b.points3D[0].z;
    }
    // TODO: Examine fitness
    // TODO: Handle vertical edge
    pcl::ModelCoefficients c;
    pcl::PointIndices line_inliers;
    pcl::SACSegmentation<pcl::PointXYZ> line_seg;
    line_seg.setOptimizeCoefficients(true);
    line_seg.setModelType(pcl::SACMODEL_LINE);
    line_seg.setMethodType(pcl::SAC_RANSAC);
    line_seg.setDistanceThreshold(boundary_ransac_thresh_);
    line_seg.setInputCloud(cloud.makeShared());
    line_seg.segment(line_inliers, c);

    // Check magnitude of the edge points in resultant model
    if (debug_fitting)
    {
      ROS_INFO_STREAM("Fit: " << line_inliers.indices.size() <<
                      " line points from " << b.points3D.size());
    }
    Eigen::Vector3f l_vector(c.values[3], c.values[4], c.values[5]);
    l_pt[0] = c.values[0];
    l_pt[1] = c.values[1];
    l_pt[2] = c.values[2];
    return l_vector;
  }

  Eigen::Vector4f splitPlaneVertical(Boundary& b, bool debug_fitting=false)
  {
    Eigen::Vector3f l_pt;
    Eigen::Vector3f l_dir = getRANSACXYVector(b, l_pt);
    l_dir /= l_dir.norm();
    const Eigen::Vector3f z_axis(0, 0, 1);
    Eigen::Vector3f n = l_dir.cross(z_axis);
    n = n/n.norm();
    float p = -(n[0]*l_pt[0]+n[1]*l_pt[1]+n[2]*l_pt[2]);
    Eigen::Vector4f hessian(n[0], n[1], n[2], p);
    if (debug_fitting)
    {
      for (unsigned int i = 0; i < b.points3D.size(); ++i)
      {
        pcl::PointXYZ x = b.points3D[i];
        const float D = hessian[0]*x.x + hessian[1]*x.y + hessian[2]*x.z +
            hessian[3];
        ROS_INFO_STREAM("Point: (" << x.x << ", " << x.y << ", " << x.z <<
                        ") : " << D);
      }
      ROS_INFO_STREAM("hessian is: " << hessian[0] << ", " << hessian[1] <<
                      ", " <<  hessian[2] << ", " << hessian[3] << ")");
    }
    return hessian;
  }

  std::vector<unsigned int> checkSingulated(std::vector<Boundary>& boundaries,
                                   ProtoObjects& objs)
  {
    std::set<unsigned int> has_pushes;
    for (unsigned int b = 0; b < boundaries.size(); ++b)
    {
      // Ignore small boundaries or those not on objects
      if (boundaries[b].object_id == objs.size() ||
          boundaries[b].external || boundaries[b].too_short)
      {
        continue;
      }
      const int i = boundaries[b].object_id;
      const unsigned int bin = quantizeAngle(boundaries[b].ort);
      // Only choose from boundaries associated with unpushed directions
      if (objs[i].push_history[bin] == 0)
      {
        has_pushes.insert(i);
      }
    }
    std::vector<unsigned int> singulated;
    for (unsigned int i = 0; i < objs.size(); ++i)
    {
      if (has_pushes.count(i) == 0)
      {
        singulated.push_back(i);
      }
    }
    return singulated;
  }

  Boundary chooseTestBoundary(std::vector<Boundary>& boundaries,
                              ProtoObjects& objs)
  {
    ROS_INFO_STREAM("chooseTestBoundary()");
    std::vector<Boundary> possible_boundaries;
    for (unsigned int b = 0; b < boundaries.size(); ++b)
    {
      // Ignore small boundaries or those not on objects
      if (boundaries[b].object_id == objs.size() ||
          boundaries[b].external || boundaries[b].too_short)
      {
        continue;
      }
      const int i = boundaries[b].object_id;
      const unsigned int bin = quantizeAngle(boundaries[b].ort);
      // Only choose from boundaries associated with unpushed directions
      if (objs[i].push_history[bin] == 0)
      {
        possible_boundaries.push_back(boundaries[b]);
      }
    }
    ROS_INFO_STREAM("Getting random test boundary from set of " <<
                    possible_boundaries.size() << " possible boundaries.");
    int chosen_id = chooseRandTestBoundary(possible_boundaries, objs);
    if ( chosen_id < 0 || chosen_id > possible_boundaries.size())
    {
      Boundary b;
      b.object_id = -1;
      return b;
    }
    ROS_INFO_STREAM("Chose boundary: " << chosen_id << " has objet_id: " <<
                     possible_boundaries[chosen_id].object_id);
    ROS_INFO_STREAM("Boundary.ort is: " << possible_boundaries[chosen_id].ort);
    return possible_boundaries[chosen_id];
  }

  int chooseRandTestBoundary(std::vector<Boundary>& boundaries,
                             ProtoObjects& objs)
  {
    std::vector<double> scores(boundaries.size(), 0.0f);
    for (unsigned int i = 0; i < boundaries.size(); ++i)
    {
      if (boundaries[i].object_id >= objs.size() || boundaries[i].too_short ||
          boundaries[i].external)
      {
        continue;
      }
      else
      {
        scores[i] = 1.0;
      }
    }
    int chosen_id = sampleScore(scores);
    return chosen_id;
  }

  PushVector determinePushDirection(double push_dist,
                                    double push_angle,
                                    ProtoTabletopObject& split0,
                                    ProtoTabletopObject& split1,
                                    ProtoObjects& objs, unsigned int id,
                                    cv::Mat& lbl_img, Boundary& boundary)
  {
    double push_angle_pos = 0.0;
    double push_angle_neg = 0.0;
    if (push_angle > 0.0)
    {
      push_angle_pos = push_angle;
      push_angle_neg = push_angle - M_PI;
    }
    else
    {
      push_angle_neg = push_angle;
      push_angle_pos = push_angle + M_PI;
    }
    const Eigen::Vector3f push_vec_pos(std::cos(push_angle_pos),
                                       std::sin(push_angle_pos), 0.0);
    const Eigen::Vector3f push_vec_neg(std::cos(push_angle_neg),
                                       std::sin(push_angle_neg), 0.0);

    std::vector<PushOpt> split_opts;
    split_opts.push_back(PushOpt(split0, push_angle_pos, push_vec_pos, id, 0,
                                 push_dist));
    split_opts.push_back(PushOpt(split0, push_angle_neg, push_vec_neg, id, 0,
                                 push_dist));
    split_opts.push_back(PushOpt(split1, push_angle_pos, push_vec_pos, id, 1,
                                 push_dist));
    split_opts.push_back(PushOpt(split1, push_angle_neg, push_vec_neg, id, 1,
                                 push_dist));

    // TODO: Make pushes not cause collisions between objects
    // Choose pushing location closer to the robot by examining the 4
    // possible intersections with the boundaries
    XYZPointCloud s0_int = pcl_segmenter_->lineCloudIntersection(
        split0.cloud, push_vec_pos, split0.centroid);
    XYZPointCloud s1_int = pcl_segmenter_->lineCloudIntersection(
        split1.cloud, push_vec_pos, split1.centroid);
    unsigned int chosen_idx = split_opts.size();
    bool p_not_set = true;
    geometry_msgs::Point p;
    for (unsigned int i = 0; i < split_opts.size(); ++i)
    {
      if (split_opts[i].push_angle < min_push_angle_ ||
          split_opts[i].push_angle > max_push_angle_) continue;
      geometry_msgs::Point p_cur;
      if (split_opts[i].split_id == 0 && s0_int.size() > 0)
      {
        p_cur = determineStartPoint(s0_int, split_opts[i]);
      }
      else if (s1_int.size() > 0)
      {
        p_cur = determineStartPoint(s1_int, split_opts[i]);
      }
      else
      {
        p_cur.x = split_opts[i].obj.centroid[0];
        p_cur.y = split_opts[i].obj.centroid[1];
        p_cur.z = split_opts[i].obj.centroid[2];
      }
      if (p_not_set || p_cur.x < p.x && (p_cur.x != 0.0))
      {
        p_not_set = false;
        p = p_cur;
        chosen_idx = i;
      }
    }
    if (chosen_idx == split_opts.size() || p_not_set)
    {
      ROS_WARN_STREAM("No push direction chosen, fix this!");
      PushVector push;
      push.no_push = true;
      return push;
    }

    PushVector push;
    push.start_point = p;
    push.push_angle = split_opts[chosen_idx].push_angle;
    push.no_push = false;
    ROS_INFO_STREAM("Chosen push_dir: [" <<
                    split_opts[chosen_idx].push_unit_vec[0] << ", " <<
                    split_opts[chosen_idx].push_unit_vec[1] << ", " <<
                    split_opts[chosen_idx].push_unit_vec[2] << "]");
    ROS_INFO_STREAM("Chosen start_point: (" << p.x << ", " << p.y << ", " <<
                    p.z << ")");

#ifdef DISPLAY_PUSH_VECTOR
    ProtoObjects split_objs3D;
    split_objs3D.push_back(split0);
    split_objs3D.push_back(split1);
    cv::Mat split_img = pcl_segmenter_->projectProtoObjectsIntoImage(
        split_objs3D, lbl_img.size(), workspace_frame_);
    cv::Mat disp_img = pcl_segmenter_->displayObjectImage(
        split_img, "3D Split", false);
#ifdef DISPLAY_CHOSEN_BOUNDARY
    if (use_displays_ || write_to_disk_)
    {
      if (use_displays_)
      {
        displayBoundaryOrientation(disp_img, boundary, "chosen debug");
      }
      highlightBoundaryOrientation(disp_img, boundary, "chosen");
    }
#endif // DISPLAY_CHOSEN_BOUNDARY

    const Eigen::Vector4f moved_start = split_opts[chosen_idx].getMovedPoint(
        push.start_point);
    PointStamped start_point;
    start_point.point = push.start_point;
    start_point.header.frame_id = workspace_frame_;
    PointStamped end_point;
    end_point.point.x = moved_start[0];
    end_point.point.y = moved_start[1];
    end_point.point.z = moved_start[2];
    end_point.header.frame_id = workspace_frame_;

    cv::Point img_start_point = pcl_segmenter_->projectPointIntoImage(
        start_point);
    cv::Point img_end_point = pcl_segmenter_->projectPointIntoImage(
        end_point);
    cv::line(disp_img, img_start_point, img_end_point, cv::Scalar(0,0,1.0));
    cv::circle(disp_img, img_end_point, 4, cv::Scalar(0,1.0,0));

    if (use_displays_)
    {
      cv::imshow("push_vector", disp_img);
    }
    if (write_to_disk_)
    {
      // Write to disk to create video output
      std::stringstream push_out_name;
      push_out_name << base_output_path_ << "push_vector" << callback_count_
                    << ".tiff";
      cv::Mat push_out_img(disp_img.size(), CV_8UC3);
      disp_img.convertTo(push_out_img, CV_8UC3, 255);
      cv::imwrite(push_out_name.str(), push_out_img);
    }
#endif // DISPLAY_PUSH_VECTOR

    return push;
  }

  geometry_msgs::Point determineStartPoint(XYZPointCloud& pts, PushOpt& opt)
  {
    unsigned int min_idx = pts.size();
    unsigned int max_idx = pts.size();
    float min_y = FLT_MAX;
    float max_y = -FLT_MAX;
    // TODO: Might want to care about y if angle is close to zero
    for (unsigned int i = 0; i < pts.size(); ++i)
    {
      if (pts.at(i).y < min_y)
      {
        min_y = pts.at(i).y;
        min_idx = i;
      }
      if (pts.at(i).y > max_y)
      {
        max_y = pts.at(i).y;
        max_idx = i;
      }
    }

    geometry_msgs::Point p;
    // NOTE: greater than 0 implies right arm, pushing to the left, want right
    // extreme, means min y value
    if (opt.push_angle > 0)
    {
      p.x = pts.at(min_idx).x;
      p.y = pts.at(min_idx).y;
      p.z = pts.at(min_idx).z;
    }
    else
    {
      p.x = pts.at(max_idx).x;
      p.y = pts.at(max_idx).y;
      p.z = pts.at(max_idx).z;
    }

    return p;
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
    double max_clearance = 0.0;
    const Eigen::Vector4f moved_cent = split.getMovedCentroid();
    for (unsigned int i = 0; i < objs.size(); ++i)
    {
      // Don't compare object to itself
      if (i == split.object_id) continue;

      const double clearance = (moved_cent - objs[i].centroid).norm();

      if (clearance < min_clearance)
      {
        min_clearance = clearance;
      }
      if (clearance > max_clearance)
      {
        max_clearance = clearance;
      }
    }
    return min_clearance;
  }

  ProtoObjects splitObject3D(Boundary& boundary, ProtoTabletopObject& to_split,
                             bool debug_fitting=false)
  {
    // Get plane containing the boundary
    Eigen::Vector4f hessian = splitPlaneVertical(boundary, debug_fitting);
    // Split based on the plane
    return splitObject3D(hessian, to_split, debug_fitting);
  }

  ProtoObjects splitObject3D(Eigen::Vector4f& hessian,
                             ProtoTabletopObject& to_split,
                             bool debug_fitting=false)
  {
    // Split the point clouds based on the half plane distance test
    pcl::PointIndices p1;
    for (unsigned int i = 0; i < to_split.cloud.size(); ++i)
    {
      const pcl::PointXYZ x = to_split.cloud.at(i);
      const float D = hessian[0]*x.x + hessian[1]*x.y + hessian[2]*x.z +
          hessian[3];
      if (D > 0)
      {
        p1.indices.push_back(i);
      }
    }
    // Extract indices
    ProtoObjects split;
    ProtoTabletopObject po1;
    ProtoTabletopObject po2;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
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

    if (debug_fitting)
    {
      pcl::PointCloud<pcl::PointXYZI> label_cloud;
      label_cloud.header.frame_id = workspace_frame_;
      label_cloud.resize(po1.cloud.size()+po2.cloud.size());
      for (unsigned int i = 0, k =0; i < split.size(); ++i)
      {
        for (unsigned int j = 0; j < split[i].cloud.size(); ++j, ++k)
        {
          // NOTE: Intensity 0 is the table; so use 1-based indexing
          pcl::PointXYZI p;
          p.x = split[i].cloud[j].x;
          p.y = split[i].cloud[j].y;
          p.z = split[i].cloud[j].z;
          p.intensity = i;
          label_cloud.at(k) = p;
        }
      }
      sensor_msgs::PointCloud2 label_cloud_msg;
      pcl::toROSMsg(label_cloud, label_cloud_msg);
      obj_push_pub_.publish(label_cloud_msg);
    }

    return split;
  }

  int sampleScore(std::vector<double>& scores)
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
    if (samples.size() < 1)
    {
      if (scores.size() > 0)
      {
        ROS_WARN_STREAM("No non-zero sample scores, returning randomly from score list");
        return (rand() % scores.size());
      }
      else
      {
        ROS_WARN_STREAM("No samples");
        return -1;
      }
    }
    ROS_DEBUG_STREAM("Sampling from list of size: " << samples.size());
    if (samples.size() == 1)
    {
      return samples[0].id;
    }

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
  void displayBoundaryOrientation(cv::Mat& obj_img, Boundary& boundary,
                                  std::string title)
  {
    cv::Mat obj_disp_img(obj_img.size(), CV_32FC3);
    if (obj_img.depth() == CV_8U)
    {
      cv::Mat obj_img_f;
      obj_img.convertTo(obj_img_f, CV_32FC1, 30.0/255);
      cv::cvtColor(obj_img_f, obj_disp_img, CV_GRAY2BGR);
    }
    else if (obj_img.type() == CV_32FC1)
    {
      cv::cvtColor(obj_img, obj_disp_img, CV_GRAY2BGR);
    }
    else
    {
      obj_img.copyTo(obj_disp_img);
    }
    Eigen::Vector3f l_pt;
    Eigen::Vector3f l_dir = getRANSACXYVector(boundary, l_pt);
    Eigen::Vector4f n = splitPlaneVertical(boundary);
    Eigen::Vector4f table_centroid = pcl_segmenter_->getTableCentroid();
    const cv::Scalar red(0.0f, 0.0f, 1.0f);
    const cv::Scalar blue(1.0f, 0.0f, 0.0f);
    const cv::Scalar cyan(1.0f, 1.0f, 0.0f);
    const cv::Scalar green(0.0f, 1.0f, 0.0f);
    for (unsigned int i = 0; i < boundary.points3D.size(); ++i)
    {
      PointStamped start_pt;
      start_pt.header.frame_id = workspace_frame_;
      start_pt.point.x = boundary.points3D[i].x;
      start_pt.point.y = boundary.points3D[i].y;
      start_pt.point.z = boundary.points3D[i].z;
      PointStamped end_pt;
      end_pt.header.frame_id = workspace_frame_;
      end_pt.point.x = start_pt.point.x + n[0]*0.10;
      end_pt.point.y = start_pt.point.y + n[1]*0.10;
      end_pt.point.z = start_pt.point.z + n[2]*0.10;

      cv::Point img_start_pt = pcl_segmenter_->projectPointIntoImage(start_pt);
      cv::Point img_end_pt = pcl_segmenter_->projectPointIntoImage(end_pt);
      cv::line(obj_disp_img, img_start_pt, img_end_pt, red);
      cv::circle(obj_disp_img, img_end_pt, 4, blue);
    }
    PointStamped l_point;
    l_point.header.frame_id = workspace_frame_;
    l_point.point.x = l_pt[0];
    l_point.point.y = l_pt[1];
    l_point.point.z = l_pt[2];
    PointStamped l_end;
    l_end.header.frame_id = workspace_frame_;
    l_end.point.x = l_pt[0] + l_dir[0]*0.10;
    l_end.point.y = l_pt[1] + l_dir[1]*0.10;
    l_end.point.z = l_pt[2] + l_dir[2]*0.10;
    cv::Point img_l_pt = pcl_segmenter_->projectPointIntoImage(l_point);
    cv::Point img_l_end = pcl_segmenter_->projectPointIntoImage(l_end);
    cv::circle(obj_disp_img, img_l_pt, 6, cyan);
    cv::line(obj_disp_img, img_l_pt, img_l_end, green);
    const cv::Vec3f green_v(0.0f, 1.0f, 0.0f);
    for (unsigned int i = 0; i < boundary.size(); ++i)
    {
      obj_disp_img.at<cv::Vec3f>(boundary[i].y, boundary[i].x) = green_v;
    }
    cv::imshow(title, obj_disp_img);
  }

  void highlightBoundaryOrientation(cv::Mat& obj_img, Boundary& boundary,
                                  std::string title)
  {
    cv::Mat obj_disp_img(obj_img.size(), CV_32FC3);
    if (obj_img.depth() == CV_8U)
    {
      cv::Mat obj_img_f;
      obj_img.convertTo(obj_img_f, CV_32FC1, 30.0/255);
      cv::cvtColor(obj_img_f, obj_disp_img, CV_GRAY2BGR);
    }
    else if (obj_img.type() == CV_32FC1)
    {
      cv::cvtColor(obj_img, obj_disp_img, CV_GRAY2BGR);
    }
    else
    {
      obj_img.copyTo(obj_disp_img);
    }
    cv::Vec3f green(0.0,1.0,0.0);
    for (unsigned int i = 0; i < boundary.size(); ++i)
    {
      obj_disp_img.at<cv::Vec3f>(boundary[i].y, boundary[i].x) = green;
    }
    if (use_displays_)
    {
      cv::imshow(title, obj_disp_img);
    }
    if (write_to_disk_)
    {
      // Write to disk to create video output
      std::stringstream bound_out_name;
      bound_out_name << base_output_path_ << "chosen" << callback_count_
                     << ".tiff";
      cv::Mat bound_out_img(obj_disp_img.size(), CV_8UC3);
      obj_disp_img.convertTo(bound_out_img, CV_8UC3, 255);
      cv::imwrite(bound_out_name.str(), bound_out_img);
    }
  }

  void draw3DBoundaries(std::vector<Boundary> boundaries, cv::Mat& img,
                        unsigned int objs_size, bool is_obj_img=false)
  {
    cv::Mat disp_img(img.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));

    if (is_obj_img)
    {
      for (int r = 0; r < img.rows; ++r)
      {
        for (int c = 0; c < img.cols; ++c)
        {
          unsigned int id = img.at<uchar>(r,c);
          if (id > 0)
          {
            disp_img.at<cv::Vec3f>(r,c) = pcl_segmenter_->colors_[id-1];
          }
        }
      }
    }
    else
    {
      if (img.type() == CV_32FC3)
      {
        img.copyTo(disp_img);
      }
      else if (img.type() == CV_8UC3)
      {
        img.convertTo(disp_img, CV_32FC3, 1.0/255.0);
      }
      else if (img.type() == CV_32FC1)
      {
        cv::cvtColor(img, disp_img, CV_GRAY2BGR);
      }
      else if (img.type() == CV_8UC3)
      {
        cv::Mat tmp_img;
        img.convertTo(tmp_img, CV_32FC1, 1.0/255.0);
        cv::cvtColor(tmp_img, disp_img, CV_GRAY2BGR);
      }
    }
    cv::Vec3f no_obj(0.0, 0.0, 1.0);
    cv::Vec3f short_col(1.0, 0.0, 0.0);
    cv::Vec3f good(0.0, 1.0, 0.0);
    cv::Vec3f external(0.0, 1.0, 1.0);
    for (unsigned int b = 0; b < boundaries.size(); ++b)
    {
      cv::Vec3f color;
      if (boundaries[b].object_id < objs_size)
      {
        if (boundaries[b].too_short)
        {
          color = short_col;
        }
        else if (boundaries[b].external)
        {
          color = external;
        }
        else
        {
          color = good;
        }
      }
      else
      {
        color = no_obj;
      }
      for (unsigned int i = 0; i < boundaries[b].size(); ++i)
      {
        disp_img.at<cv::Vec3f>(boundaries[b][i].y,
                                   boundaries[b][i].x) = color;
      }
    }
    if (use_displays_)
    {
      cv::imshow("3D boundaries", disp_img);
    }
    if (write_to_disk_)
    {
      // Write to disk to create video output
      std::stringstream bound_out_name;
      bound_out_name << base_output_path_ << "bound3D" << callback_count_
                   << ".tiff";
      cv::Mat bound_out_img(disp_img.size(), CV_8UC3);
      disp_img.convertTo(bound_out_img, CV_8UC3, 255);
      cv::imwrite(bound_out_name.str(), bound_out_img);
    }
  }

  void drawObjectTransformAxises(cv::Mat& obj_img, ProtoObjects& objs)
  {
    cv::Mat obj_disp_img(obj_img.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));

    for (int r = 0; r < obj_img.rows; ++r)
    {
      for (int c = 0; c < obj_img.cols; ++c)
      {
        unsigned int id = obj_img.at<uchar>(r,c);
        if (id > 0)
        {
          obj_disp_img.at<cv::Vec3f>(r,c) = pcl_segmenter_->colors_[id-1];
        }
      }
    }
    cv::Scalar green(0.0, 1.0, 0.0);
    cv::Scalar red(0.0, 0.0, 1.0);
    cv::Scalar cyan(1.0, 1.0, 0.0);

    const Eigen::Vector4f x_axis(0.1, 0.0, 0.0, 1.0);
    const Eigen::Vector4f y_axis(0.0, 0.1, 0.0, 1.0);
    const Eigen::Vector4f z_axis(0.0, 0.0, 0.1, 1.0);
    for (unsigned int i = 0; i < objs.size(); ++i)
    {
      // Transform axises into current frame
      const Eigen::Vector4f x_t = objs[i].transform*x_axis;
      const Eigen::Vector4f y_t = objs[i].transform*y_axis;
      const Eigen::Vector4f z_t = objs[i].transform*z_axis;

      // Project axises into image
      PointStamped start_point;
      start_point.point.x = objs[i].centroid[0];
      start_point.point.y = objs[i].centroid[1];
      start_point.point.z = objs[i].centroid[2];
      start_point.header.frame_id = workspace_frame_;

      PointStamped end_point_x;
      end_point_x.point.x = start_point.point.x + x_t[0];
      end_point_x.point.y = start_point.point.y + x_t[1];
      end_point_x.point.z = start_point.point.z + x_t[2];
      end_point_x.header.frame_id = workspace_frame_;

      PointStamped end_point_y;
      end_point_y.point.x = start_point.point.x + y_t[0];
      end_point_y.point.y = start_point.point.y + y_t[1];
      end_point_y.point.z = start_point.point.z + y_t[2];
      end_point_y.header.frame_id = workspace_frame_;

      PointStamped end_point_z;
      end_point_z.point.x = start_point.point.x + z_t[0];
      end_point_z.point.y = start_point.point.y + z_t[1];
      end_point_z.point.z = start_point.point.z + z_t[2];
      end_point_z.header.frame_id = workspace_frame_;

      cv::Point img_start_point = pcl_segmenter_->projectPointIntoImage(
          start_point);
      cv::Point img_end_point_x = pcl_segmenter_->projectPointIntoImage(
          end_point_x);
      cv::Point img_end_point_y = pcl_segmenter_->projectPointIntoImage(
          end_point_y);
      cv::Point img_end_point_z = pcl_segmenter_->projectPointIntoImage(
          end_point_z);

      // Draw axises on image
      cv::line(obj_disp_img, img_start_point, img_end_point_x, red);
      cv::line(obj_disp_img, img_start_point, img_end_point_y, green);
      cv::line(obj_disp_img, img_start_point, img_end_point_z, cyan);
    }
    if (use_displays_)
    {
      cv::imshow("Object axis", obj_disp_img);
    }
    if (write_to_disk_)
    {
      // Write to disk to create video output
      std::stringstream axis_out_name;
      axis_out_name << base_output_path_ << "axis" << callback_count_
                     << ".tiff";
      cv::Mat axis_out_img(obj_disp_img.size(), CV_8UC3);
      obj_disp_img.convertTo(axis_out_img, CV_8UC3, 255);
      cv::imwrite(axis_out_name.str(), axis_out_img);
    }
  }

  int getNextID()
  {
    ROS_DEBUG_STREAM("Getting next ID: " << next_id_);
    return next_id_++;
  }

  void drawObjectHists(cv::Mat& img, ProtoObjects& objs, bool is_obj_img=false)
  {
    cv::Mat disp_img(img.size(), CV_32FC3, cv::Scalar(0.0,0.0,0.0));
    if (is_obj_img)
    {
      for (int r = 0; r < img.rows; ++r)
      {
        for (int c = 0; c < img.cols; ++c)
        {
          unsigned int id = img.at<uchar>(r,c);
          if (id > 0)
          {
            disp_img.at<cv::Vec3f>(r,c) = pcl_segmenter_->colors_[id-1];
          }
        }
      }
    }
    else
    {
      img.convertTo(disp_img, CV_32FC3, 1.0/255.0);
    }

    cv::Mat disp_img_hist;
    disp_img.copyTo(disp_img_hist);
    const Eigen::Vector4f x_axis(0.1, 0.0, 0.0, 1.0);
    // const int w = 5;
    // const int h = 30;
    const int w = histogram_bin_width_;
    const int h = histogram_bin_height_;
    const cv::Scalar est_line_color(0.0, 0.0, 0.0);
    const cv::Scalar est_fill_color(0.0, 0.0, 0.7);
    const cv::Scalar history_line_color(0.0, 0.0, 0.0);
    const cv::Scalar history_fill_color(0.0, 0.7, 0.0);
    for (unsigned int i = 0; i < objs.size(); ++i)
    {
      const Eigen::Vector4f x_t = objs[i].transform*x_axis;
      // NOTE: In degrees!
      const float start_angle = atan2(x_t[1], x_t[0])*180.0/M_PI + 180.0;
      // Get the locations from object centroids
      PointStamped center3D;
      center3D.point.x = objs[i].centroid[0];
      center3D.point.y = objs[i].centroid[1];
      center3D.point.z = objs[i].centroid[2];
      center3D.header.frame_id = workspace_frame_;
      cv::Point center = pcl_segmenter_->projectPointIntoImage(center3D);
      drawSemicircleHist(objs[i].boundary_angle_dist, disp_img, center, w, h,
                         est_line_color, est_fill_color, start_angle);
      drawSemicircleHist(objs[i].push_history, disp_img_hist, center, w, h,
                         history_line_color, history_fill_color, start_angle);
    }
    if (use_displays_)
    {
      cv::imshow("Boundary Estimate Distributions", disp_img);
      cv::imshow("Pushing History Distributions", disp_img_hist);
    }
    if (write_to_disk_)
    {
      // Write to disk to create video output
      std::stringstream est_out_name;
      std::stringstream hist_out_name;
      est_out_name << base_output_path_ << "bound_est" << callback_count_
                   << ".tiff";
      hist_out_name << base_output_path_ << "hist_est" << callback_count_
                    << ".tiff";
      cv::Mat est_out_img(disp_img.size(), CV_8UC3);
      cv::Mat hist_out_img(disp_img.size(), CV_8UC3);
      disp_img.convertTo(est_out_img, CV_8UC3, 255);
      disp_img_hist.convertTo(hist_out_img, CV_8UC3, 255);
      cv::imwrite(est_out_name.str(), est_out_img);
      cv::imwrite(hist_out_name.str(), hist_out_img);
    }
  }

  void drawSemicircleHist(std::vector<int>& hist, cv::Mat& disp_img,
                          const cv::Point center, int w, int h,
                          const cv::Scalar line_color,
                          const cv::Scalar fill_color, const float start_rot,
                          bool narrow_bins = false)
  {
    const float narrow_rad = 6.0;
    int half_circ = w*hist.size();
    const int r0 = half_circ/M_PI;
    const cv::Size s0(r0, r0);
    const float angle_inc = 180.0/(hist.size());
    float hist_max = 0.0;
    const float deg_precision = 1.0;
    for (unsigned int i = 0; i < hist.size(); ++i)
    {
      if (hist[i] > hist_max)
      {
        hist_max = hist[i];
      }
    }
    // Draw all fills
    float rot = start_rot;
    for (unsigned int i = 0; i < hist.size(); ++i)
    {
      // NOTE: need to flip histogram order to correctly display
      if (hist[hist.size()-i-1] > 0)
      {
        int d_y = h * hist[hist.size()-i-1] / hist_max;
        const int r1 = r0+d_y;
        const cv::Size s1(r1, r1);
        std::vector<cv::Point> out_arc_pts;
        float start_angle = 0.0;
        float end_angle = angle_inc;
        if (narrow_bins)
        {
          start_angle += narrow_rad;
          end_angle -= narrow_rad;
        }
        cv::ellipse2Poly(center, s1, rot, start_angle, end_angle, deg_precision,
                         out_arc_pts);
        std::vector<cv::Point> in_arc_pts;
        cv::ellipse2Poly(center, s0, rot, start_angle, end_angle, deg_precision,
                         in_arc_pts);
        std::vector<cv::Point> poly_vect;
        for (unsigned int j = 0; j < in_arc_pts.size(); ++j)
        {
          poly_vect.push_back(in_arc_pts[j]);
        }
        for (unsigned int j = out_arc_pts.size()-1; j > 0; --j)
        {
          poly_vect.push_back(out_arc_pts[j]);
        }

        int npts[1] = {poly_vect.size()};
        cv::Point* poly = new cv::Point[poly_vect.size()];
        for (unsigned int j = 0; j < poly_vect.size(); ++j)
        {
          poly[j] = poly_vect[j];
        }
        const cv::Point* pts[1] = {poly};
        // fill bin
        cv::fillPoly(disp_img, pts, npts, 1, fill_color);
        delete poly;
      }
      rot += angle_inc;
    }
    // Draw inner circle
    cv::ellipse(disp_img, center, s0, start_rot, 0.0, 180.0, line_color);

    // Draw all lines
    rot = start_rot;
    for (unsigned int i = 0; i < hist.size(); ++i)
    {
      if (hist[hist.size()-i-1] > 0)
      {
        int d_y = h * hist[hist.size()-i-1] / hist_max;
        const int r1 = r0+d_y;
        const cv::Size s1(r1, r1);
        float start_angle = 0.0;
        float end_angle = angle_inc;
        if (narrow_bins)
        {
          start_angle += narrow_rad;
          end_angle -= narrow_rad;
        }

        cv::ellipse(disp_img, center, s1, rot, start_angle, end_angle,
                    line_color);
        std::vector<cv::Point> out_arc_pts;
        cv::ellipse2Poly(center, s1, rot, start_angle, end_angle, deg_precision,
                         out_arc_pts);
        std::vector<cv::Point> in_arc_pts;
        cv::ellipse2Poly(center, s0, rot, start_angle, end_angle, deg_precision,
                         in_arc_pts);
        // Draw bin edge lines
        cv::line(disp_img, in_arc_pts.front(), out_arc_pts.front(), line_color);
        cv::line(disp_img, in_arc_pts.back(), out_arc_pts.back(), line_color);
      }
      rot += angle_inc;
    }
  }

 public:
  //
  // Getters and setters
  //

  bool isInitialized() const { return initialized_; }

  void unInitialize() { initialized_ = false; }

  //
  // Class member variables
  //
 protected:
  cv::Mat dx_kernel_;
  cv::Mat dy_kernel_;
  cv::Mat g_kernel_;
  shared_ptr<PointCloudSegmentation> pcl_segmenter_;
  XYZPointCloud prev_cloud_down_;
  XYZPointCloud prev_objs_down_;
  ProtoObjects prev_proto_objs_;
  ProtoObjects cur_proto_objs_;
  int callback_count_;
  int next_id_;
  bool initialized_;

 public:
  double min_pushing_x_;
  double max_pushing_x_;
  double min_pushing_y_;
  double max_pushing_y_;
  std::string workspace_frame_;
  ros::Publisher obj_push_pub_;
  bool threshold_edges_;
  double depth_edge_weight_;
  double edge_weight_thresh_;
  double depth_edge_weight_thresh_;
  double max_push_angle_;
  double min_push_angle_;
  double boundary_ransac_thresh_;
  int min_edge_length_;
  int num_angle_bins_;
  int num_downsamples_;
  int upscale_;
  bool use_displays_;
  bool write_to_disk_;
  int min_cluster_size_;
  int min_boundary_length_;
  std::string base_output_path_;
  int histogram_bin_width_;
  int histogram_bin_height_;
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
      it_(n),
      /*pcl_segmenter_(tf_),*/
      /*os_(pcl_segmenter_),*/
      have_depth_data_(false), tracking_(false),
      tracker_initialized_(false),
      camera_initialized_(false), tracker_count_(0), recording_input_(false)
  {
    tf_ = shared_ptr<tf::TransformListener>(new tf::TransformListener());
    pcl_segmenter_ = shared_ptr<PointCloudSegmentation>(
        new PointCloudSegmentation(tf_));
    os_ = shared_ptr<ObjectSingulation>(new ObjectSingulation(pcl_segmenter_));
    // Get parameters from the server
    n_private_.param("crop_min_x", crop_min_x_, 0);
    n_private_.param("crop_max_x", crop_max_x_, 640);
    n_private_.param("crop_min_y", crop_min_y_, 0);
    n_private_.param("crop_max_y", crop_max_y_, 480);
    n_private_.param("display_wait_ms", display_wait_ms_, 3);
    n_private_.param("use_displays", use_displays_, false);
    os_->use_displays_ = use_displays_;
    n_private_.param("write_to_disk", write_to_disk_, false);
    os_->write_to_disk_ = write_to_disk_;
    n_private_.param("min_workspace_x", min_workspace_x_, 0.0);
    n_private_.param("min_workspace_y", min_workspace_y_, 0.0);
    n_private_.param("min_workspace_z", min_workspace_z_, 0.0);
    n_private_.param("max_workspace_x", max_workspace_x_, 0.0);
    n_private_.param("max_workspace_y", max_workspace_y_, 0.0);
    n_private_.param("max_workspace_z", max_workspace_z_, 0.0);
    n_private_.param("min_pushing_x", os_->min_pushing_x_, 0.0);
    n_private_.param("min_pushing_y", os_->min_pushing_y_, 0.0);
    n_private_.param("max_pushing_x", os_->max_pushing_x_, 0.0);
    n_private_.param("max_pushing_y", os_->max_pushing_y_, 0.0);
    std::string default_workspace_frame = "/torso_lift_link";
    n_private_.param("workspace_frame", workspace_frame_,
                     default_workspace_frame);
    os_->workspace_frame_ = workspace_frame_;

    n_private_.param("threshold_edges", os_->threshold_edges_, false);
    n_private_.param("edge_weight_thresh", os_->edge_weight_thresh_, 0.5);
    n_private_.param("depth_edge_weight_thresh", os_->depth_edge_weight_thresh_,
                     0.5);
    n_private_.param("depth_edge_weight", os_->depth_edge_weight_, 0.75);
    n_private_.param("max_pushing_angle", os_->max_push_angle_, M_PI*0.5);
    n_private_.param("min_pushing_angle", os_->min_push_angle_, -M_PI*0.5);
    n_private_.param("boundary_ransac_thresh", os_->boundary_ransac_thresh_,
                     0.01);
    n_private_.param("min_edge_length", os_->min_edge_length_, 3);
    n_private_.param("num_angle_bins", os_->num_angle_bins_, 8);
    n_private_.param("os_min_cluster_size", os_->min_cluster_size_, 20);
    n_private_.param("os_min_boundary_length", os_->min_boundary_length_, 3);
    // NOTE: Must be at least 3 for mathematical reasons
    os_->min_boundary_length_ = std::max(os_->min_boundary_length_, 3);
    n_private_.param("os_hist_bin_width", os_->histogram_bin_width_, 5);
    n_private_.param("os_hist_bin_height", os_->histogram_bin_height_, 30);
    std::string output_path_def = "~";
    n_private_.param("img_output_path", base_output_path_, output_path_def);
    os_->base_output_path_ = base_output_path_;

    n_private_.param("min_table_z", pcl_segmenter_->min_table_z_, -0.5);
    n_private_.param("max_table_z", pcl_segmenter_->max_table_z_, 1.5);
    pcl_segmenter_->min_workspace_x_ = min_workspace_x_;
    pcl_segmenter_->max_workspace_x_ = max_workspace_x_;
    pcl_segmenter_->min_workspace_z_ = min_workspace_z_;
    pcl_segmenter_->max_workspace_z_ = max_workspace_z_;

    n_private_.param("autostart_tracking", tracking_, false);
    n_private_.param("autostart_pcl_segmentation", autorun_pcl_segmentation_,
                     false);
    n_private_.param("use_guided_pushes", use_guided_pushes_, true);

    n_private_.param("num_downsamples", num_downsamples_, 2);
    pcl_segmenter_->num_downsamples_ = num_downsamples_;
    os_->num_downsamples_ = num_downsamples_;
    os_->upscale_ = std::pow(2,num_downsamples_);

    std::string cam_info_topic_def = "/kinect_head/rgb/camera_info";
    n_private_.param("cam_info_topic", cam_info_topic_,
                     cam_info_topic_def);
    n_private_.param("table_ransac_thresh", pcl_segmenter_->table_ransac_thresh_,
                     0.01);
    n_private_.param("table_ransac_angle_thresh",
                     pcl_segmenter_->table_ransac_angle_thresh_, 30.0);
    n_private_.param("pcl_cluster_tolerance", pcl_segmenter_->cluster_tolerance_,
                     0.25);
    n_private_.param("pcl_difference_thresh", pcl_segmenter_->cloud_diff_thresh_,
                     0.01);
    n_private_.param("pcl_min_cluster_size", pcl_segmenter_->min_cluster_size_,
                     100);
    n_private_.param("pcl_max_cluster_size", pcl_segmenter_->max_cluster_size_,
                     2500);
    n_private_.param("pcl_voxel_downsample_res", pcl_segmenter_->voxel_down_res_,
                     0.005);
    n_private_.param("pcl_cloud_intersect_thresh",
                     pcl_segmenter_->cloud_intersect_thresh_, 0.005);
    n_private_.param("pcl_concave_hull_alpha", pcl_segmenter_->hull_alpha_,
                     0.1);
    n_private_.param("use_pcl_voxel_downsample", pcl_segmenter_->use_voxel_down_,
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
    pcl_segmenter_->pcl_obj_seg_pub_ = n_.advertise<sensor_msgs::PointCloud2>(
        "separate_table_objs", 1000);
    pcl_segmenter_->pcl_down_pub_ = n_.advertise<sensor_msgs::PointCloud2>(
        "downsampled_objs", 1000);
    os_->obj_push_pub_ = n_.advertise<sensor_msgs::PointCloud2>(
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
      pcl_segmenter_->cam_info_ = cam_info_;
    }
    // Convert images to OpenCV format
    cv::Mat color_frame(bridge_.imgMsgToCv(img_msg));
    cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));

    // Swap kinect color channel order
    cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);

    // Transform point cloud into the correct frame and convert to PCL struct
    XYZPointCloud cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    tf_->waitForTransform(workspace_frame_, cloud.header.frame_id,
                          cloud.header.stamp, ros::Duration(0.5));
    pcl_ros::transformPointCloud(workspace_frame_, cloud, cloud, *tf_);

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
    pcl_segmenter_->cur_camera_header_ = cur_camera_header_;

    // Debug stuff
    if (autorun_pcl_segmentation_)
    {
      getPushPose(default_push_dist_, use_guided_pushes_);
      if (!os_->isInitialized())
      {
        ROS_INFO_STREAM("Calling initialize.");
        os_->initialize(cur_color_frame_, cur_depth_frame_, cur_point_cloud_,
                        cur_workspace_mask_);
      }
    }

    // Display junk
#ifdef DISPLAY_INPUT_COLOR
    if (use_displays_)
    {
      cv::imshow("color", cur_color_frame_);
    }
    // TODO: Way too much disk writing!
    if (write_to_disk_ && recording_input_)
    {
      std::stringstream color_out_name;
      color_out_name << base_output_path_ << "video/input" << tracker_count_
                     << ".tiff";
      tracker_count_++;
      cv::imwrite(color_out_name.str(), cur_color_frame_);
    }
#endif // DISPLAY_INPUT_COLOR
#ifdef DISPLAY_INPUT_DEPTH
    if (use_displays_)
    {
      double depth_max = 1.0;
      cv::minMaxLoc(cur_depth_frame_, NULL, &depth_max);
      cv::Mat depth_display = cur_depth_frame_.clone();
      depth_display /= depth_max;
      cv::imshow("input_depth", depth_display);
    }
#endif // DISPLAY_INPUT_DEPTH
#ifdef DISPLAY_WORKSPACE_MASK
    if (use_displays_)
    {
      cv::imshow("workspace_mask", cur_workspace_mask_);
    }
#endif // DISPLAY_WORKSPACE_MASK
#ifdef DISPLAY_WAIT
    if (use_displays_)
    {
      cv::waitKey(display_wait_ms_);
    }
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
      if (req.initialize)
      {
        tracker_count_ = 0;
        os_->unInitialize();
        os_->initialize(cur_color_frame_, cur_depth_frame_,
                        cur_point_cloud_, cur_workspace_mask_);
        res.no_push = true;
        recording_input_ = false;
        return true;
      }
      else
      {
        if ( ! recording_input_)
        {
          recording_input_ = true;
        }
        if ( !req.no_push_calc)
        {
          recording_input_ = false;
        }
        res = getPushPose(req.push_dist, req.use_guided, req.no_push_calc);
      }
    }
    else
    {
      ROS_ERROR_STREAM("Calling getPushPose prior to receiving sensor data.");
      res.no_push = true;
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
  PushVector getPushPose(double push_dist=0.1, bool use_guided=true,
                         bool no_push_calc=false)
  {
    if (!use_guided)
    {
      return os_->findRandomPushPose(cur_point_cloud_);
    }
    else
    {
      return os_->getPushVector(push_dist, no_push_calc,
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
    Eigen::Vector4f table_centroid = pcl_segmenter_->getTablePlane(cloud,
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
  shared_ptr<tf::TransformListener> tf_;
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
  shared_ptr<PointCloudSegmentation> pcl_segmenter_;
  shared_ptr<ObjectSingulation> os_;
  bool have_depth_data_;
  int crop_min_x_;
  int crop_max_x_;
  int crop_min_y_;
  int crop_max_y_;
  int display_wait_ms_;
  bool use_displays_;
  bool write_to_disk_;
  std::string base_output_path_;
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
  bool recording_input_;
};

int main(int argc, char ** argv)
{
  int seed = time(NULL);
  srand(seed);
  std::cout << "Rand seed is: " << seed << std::endl;
  ros::init(argc, argv, "object_singulation_node");
  ros::NodeHandle n;
  ObjectSingulationNode singulation_node(n);
  singulation_node.spin();
  return 0;
}
