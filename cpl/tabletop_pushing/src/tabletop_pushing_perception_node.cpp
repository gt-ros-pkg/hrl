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
#include <actionlib/server/simple_action_server.h>

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
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// cpl_visual_features
#include <cpl_visual_features/motion/flow_types.h>
#include <cpl_visual_features/motion/dense_lk.h>
#include <cpl_visual_features/motion/feature_tracker.h>

// tabletop_pushing
#include <tabletop_pushing/PushPose.h>
#include <tabletop_pushing/LocateTable.h>
#include <tabletop_pushing/ObjectSingulationAction.h>

#include <tabletop_pushing/extern/graphcut/graph.h>
#include <tabletop_pushing/extern/graphcut/energy.h>
#include <tabletop_pushing/extern/graphcut/GCoptimization.h>

// STL
#include <vector>
#include <deque>
#include <queue>
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
// #define DISPLAY_INPUT_COLOR 1
// #define DISPLAY_INPUT_DEPTH 1
#define DISPLAY_WORKSPACE_MASK 1
// #define DISPLAY_OPTICAL_FLOW 1
// #define DISPLAY_PLANE_ESTIMATE 1
// #define DISPLAY_GRAPHCUT 1
// #define VISUALIZE_GRAPH_WEIGHTS 1
// #define VISUALIZE_GRAPH_EDGE_WEIGHTS 1
// #define VISUALIZE_ARM_GRAPH_WEIGHTS 1
// #define VISUALIZE_ARM_GRAPH_EDGE_WEIGHTS 1
// #define DISPLAY_ARM_CIRCLES 1
// #define DISPLAY_TABLE_DISTANCES 1
// #define DISPLAY_FLOW_FIELD_CLUSTERING 1
#define DISPLAY_OBJECT_BOUNDARIES 1
// #define WRITE_INPUT_TO_DISK 1
// #define WRITE_CUTS_TO_DISK 1
// #define WRITE_FLOWS_TO_DISK 1
// #define WRITE_ARM_CUT_TO_DISK 1

// Functional IFDEFS
#define MEDIAN_FILTER_FLOW 1
#define USE_WORKSPACE_MASK_FOR_ARM 1
// #define USE_TABLE_COLOR_ESTIMATE 1

using tabletop_pushing::PushPose;
using tabletop_pushing::LocateTable;
using geometry_msgs::PoseStamped;
using geometry_msgs::PointStamped;
using geometry_msgs::Pose2D;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2> MySyncPolicy;
typedef Graph<float, float, float> GraphType;
typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

using cpl_visual_features::AffineFlowMeasure;
using cpl_visual_features::AffineFlowMeasures;
using cpl_visual_features::DenseLKFlow;
using cpl_visual_features::FeatureTracker;
using cpl_visual_features::Descriptor;

inline float max(const float a, const double b)
{
  return std::max(static_cast<double>(a), b);
}

inline float min(const float a, const double b)
{
  return std::min(static_cast<double>(a), b);
}

class ArmModel
{
 public:
  ArmModel() : l_hand_on(false),  r_hand_on(false), l_arm_on(false),
               r_arm_on(false)
  {
    hands.clear();
    arms.clear();
  }
  unsigned int size()
  {
    return 2;
  }

  std::vector<cv::Point> operator[](unsigned int i)
  {
    if (i == 0)
      return hands;
    if (i > 1)
    {
      std::stringstream err_msg;
      err_msg << "Index argument: " << i << " is out of range in class ArmModel";
      throw std::out_of_range(err_msg.str());
    }
    return arms;
  }

  /**
   * Return the distance to the closest point on the arm from an image point p
   *
   * @param p The point in the image
   *
   * @return The distance to the closest point on any in-image arm
   */
  float distanceToArm(cv::Point2f p, cv::Mat& depth_frame)
  {
    float l_dist = distanceToArm(p, l_chain, depth_frame);
    float r_dist = distanceToArm(p, r_chain, depth_frame);
    if (l_dist < 0 && r_dist < 0) return -1.0f;
    if (l_dist < 0)
    {
      return r_dist;
    }
    if (r_dist < 0)
    {
      return l_dist;
    }
    return min(l_dist, r_dist);
  }

  float distanceToArm(cv::Point2f p, std::vector<cv::Point>& chain,
                      cv::Mat& depth_frame)
  {
    if (chain.size() == 0)
    {
      return -1.0f;
    }
    float min_dist = 640.0f*480.0f;
    for (unsigned int i = 1; i < chain.size(); ++i)
    {
      float d_i = pointLineDistance(p, chain[i-1] , chain[i], depth_frame);
      if (d_i < min_dist)
        min_dist = d_i;
    }
    return min_dist;
  }

  float pointLineDistance(cv::Point2f p, cv::Point2f l0, cv::Point2f l1,
                          cv::Mat& depth_frame)
  {
    if (l0.x == l1.x)
    {
      cv::Point2f q(l0.x, p.y);
      float l_max_x = max(l0.x, l1.x);
      float l_min_x = min(l0.x, l1.x);
      if (p.y > l_max_x)
      {
        q.y = l_max_x;
      }
      else if (p.y < l_min_x)
      {
        q.y = l_min_x;
      }
      return pointPointDistance(p,q,depth_frame);
    }
    cv::Point2f x0;
    cv::Point2f x1;

    if (l0.x < l1.x)
    {
      x0 = l0;
      x1 = l1;
    }
    else
    {
      x0 = l1;
      x1 = l0;
    }
    cv::Point2f v = x1 - x0;
    cv::Point2f w = p - x0;

    float c0 = w.x*v.x+w.y*v.y;
    float c1 = v.x*v.x+v.y*v.y;
    float b = c0/c1;

    cv::Point2f q = x0 + b*v;

    float d = pointPointDistance(p,q,depth_frame);
    if (c0 <= 0 || q.x < x0.x)
    {
      d = pointPointDistance(p,x0,depth_frame);
    }
    if (c1 <= 0 || q.x > x1.x)
    {
      d = pointPointDistance(p,x1,depth_frame);
    }
    return d;
  }

  float pointPointDistance(cv::Point2f& p, cv::Point2f& q, cv::Mat& depth_frame)
  {
    // return hypot(p.x-q.x,p.y-q.y);
    // TODO: Use 3D distance between the two points
    const float d_x = p.x-q.x;
    const float d_y = p.y-q.y;
    // const float d_d = (depth_frame.at<float>(p.y,p.x)-
    //                    depth_frame.at<float>(q.y,q.x));
    // return sqrt(d_x*d_x + d_y*d_y + d_d*d_d);
    return sqrt(d_x*d_x + d_y*d_y);
  }

  std::vector<cv::Point> hands;
  std::vector<cv::Point> arms;
  std::vector<cv::Point> l_chain;
  std::vector<cv::Point> r_chain;
  bool l_hand_on;
  bool r_hand_on;
  bool l_arm_on;
  bool r_arm_on;
};

class ProtoTabletopObject
{
 public:
  XYZPointCloud cloud;
  Eigen::Vector4f centroid;
  Eigen::Vector4f table_centroid;
  // TODO: Add normals?
  // TODO: Add tracking features
  int id;
};

typedef std::deque<ProtoTabletopObject> ProtoObjects;

class PointCloudSegmentation
{
 public:
  PointCloudSegmentation(FeatureTracker* ft) : have_cur_cloud_(false), ft_(ft)
  {
  }

  /**
   * Function to determine the table plane in a point cloud
   *
   * @param cloud The cloud with the table as dominant plane.
   *
   * @return The centroid of the points belonging to the table plane.
   */
  Eigen::Vector4f getTablePlane(XYZPointCloud& cloud)
  {
    XYZPointCloud cloud_downsampled;
    if (use_voxel_down_)
    {
      pcl::VoxelGrid<pcl::PointXYZ> downsample;
      downsample.setInputCloud(
          boost::make_shared<XYZPointCloud>(cloud));
      downsample.setLeafSize(voxel_down_res_, voxel_down_res_, voxel_down_res_);
      downsample.filter(cloud_downsampled);
    }

    // Filter Cloud to not look for table planes on the ground
    XYZPointCloud cloud_z_filtered, cloud_filtered;
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    if (use_voxel_down_)
    {
      z_pass.setInputCloud(
          boost::make_shared<XYZPointCloud>(cloud_downsampled));
    }
    else
    {
      z_pass.setInputCloud(
          boost::make_shared<XYZPointCloud>(cloud));
    }
    z_pass.setFilterFieldName("z");
    z_pass.setFilterLimits(min_table_z_, max_table_z_);
    z_pass.filter(cloud_z_filtered);

    // Filter to be just in the range in front of the robot
    pcl::PassThrough<pcl::PointXYZ> x_pass;
    x_pass.setInputCloud(
        boost::make_shared<XYZPointCloud >(cloud_z_filtered));
    x_pass.setFilterFieldName("x");
    x_pass.setFilterLimits(min_workspace_x_, max_workspace_x_);
    x_pass.filter(cloud_filtered);

    // Segment the tabletop from the points using RANSAC plane fitting
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices plane_inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold (table_ransac_thresh_);
    plane_seg.setInputCloud(
        boost::make_shared<XYZPointCloud>(cloud_filtered));
    plane_seg.segment(plane_inliers, coefficients);
    pcl::copyPointCloud(cloud_filtered, plane_inliers, cur_plane_cloud_);
    // Extract the outliers from the point clouds
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    XYZPointCloud objects_cloud;
    pcl::PointIndices plane_outliers;
    extract.setInputCloud(
        boost::make_shared<XYZPointCloud > (cloud_filtered));
    extract.setIndices(boost::make_shared<pcl::PointIndices> (plane_inliers));
    extract.setNegative(true);
    extract.filter(cur_objs_cloud_);
    // Extract the plane members into their own point cloud
    Eigen::Vector4f table_centroid;
    pcl::compute3DCentroid(cur_plane_cloud_, table_centroid);
    have_cur_cloud_ = true;
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
                                   bool extract_table=true)
  {
    // Get table plane
    if (extract_table)
    {
      table_centroid_ = getTablePlane(input_cloud);
    }

    // TODO: Move downsampling into its own function
    // XYZPointCloud objects_cloud_down = downsampleCloud(input_cloud);

    // Remove points below the table plane and downsample before continuing
    XYZPointCloud objects_z_filtered, objects_cloud_down;
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setFilterFieldName("z");
    ROS_DEBUG_STREAM("Number of points in cur_objs_cloud_ is: " <<
                     cur_objs_cloud_.size());
    z_pass.setInputCloud(boost::make_shared<XYZPointCloud>(cur_objs_cloud_));
    z_pass.setFilterLimits(table_centroid_[2], max_table_z_);
    z_pass.filter(objects_z_filtered);
    ROS_DEBUG_STREAM("Number of points in objs_z_filtered is: " <<
                     objects_z_filtered.size());

    pcl::VoxelGrid<pcl::PointXYZ> downsample_outliers;
    downsample_outliers.setInputCloud(
        boost::make_shared<XYZPointCloud>(objects_z_filtered));
    downsample_outliers.setLeafSize(voxel_down_res_, voxel_down_res_,
                                    voxel_down_res_);
    downsample_outliers.filter(objects_cloud_down);
    ROS_DEBUG_STREAM("Number of points in objs_downsampled: " <<
                     objects_cloud_down.size());

    // Find independent regions
    // NOTE: Currently not printing the objects here, but in differencing
    ProtoObjects objs = clusterProtoObjects(objects_cloud_down, true);
    updatePreviousCloud(objects_cloud_down);
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
    pcl_cluster.setInputCloud(
        boost::make_shared<XYZPointCloud>(objects_cloud));
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
      objs.push_back(po);
    }
    return objs;
  }

  ProtoObjects pointCloudDifference(XYZPointCloud& prev_cloud,
                                    XYZPointCloud& cur_cloud)
  {
    // TODO: Make sure point clouds are downsampled
    pcl::SegmentDifferences<pcl::PointXYZ> pcl_diff;
    pcl_diff.setDistanceThreshold(cloud_diff_thresh_);
    pcl_diff.setInputCloud(boost::make_shared<XYZPointCloud>(prev_cloud));
    pcl_diff.setTargetCloud(boost::make_shared<XYZPointCloud>(cur_cloud));
    XYZPointCloud cloud_out;
    ROS_INFO_STREAM("Differencing clouds.");
    pcl_diff.segment(cloud_out);
    ROS_INFO_STREAM("Differenced clouds.");
    ROS_INFO_STREAM("cloud_out.size(): " << cloud_out.size());
    // TODO: Debug / display these output regions
    ROS_INFO_STREAM("Clustering proto objs.");
    ProtoObjects moved = clusterProtoObjects(cloud_out, true);
    ROS_INFO_STREAM("moved.size(): " << moved.size());
    ROS_INFO_STREAM("Clustered proto objs.");
    return moved;
  }

  XYZPointCloud downsampleCloud(XYZPointCloud& cloud_in)
  {
    XYZPointCloud cloud_z_filtered, cloud_down;
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setFilterFieldName("z");
    ROS_DEBUG_STREAM("Number of points in cloud_in is: " <<
                     cloud_in.size());
    z_pass.setInputCloud(boost::make_shared<XYZPointCloud>(cloud_in));
    z_pass.setFilterLimits(table_centroid_[2], max_table_z_);
    z_pass.filter(cloud_z_filtered);
    ROS_DEBUG_STREAM("Number of points in cloud_z_filtered is: " <<
                     cloud_z_filtered.size());

    pcl::VoxelGrid<pcl::PointXYZ> downsample_outliers;
    downsample_outliers.setInputCloud(
        boost::make_shared<XYZPointCloud>(cloud_z_filtered));
    downsample_outliers.setLeafSize(voxel_down_res_, voxel_down_res_,
                                    voxel_down_res_);
    downsample_outliers.filter(cloud_down);
    ROS_DEBUG_STREAM("Number of points in objs_downsampled: " <<
                     cloud_down.size());
    return cloud_down;
  }

  void updatePreviousCloud(XYZPointCloud& cur_update)
  {
    XYZPointCloud to_prev(cur_update);
    prev_objs_cloud_ = to_prev;
    ROS_INFO_STREAM("Updated previous cloud.");
  }

  ProtoObjects updateObjects(XYZPointCloud& cur_cloud)
  {
    // TODO: Downsample cloud
    ROS_INFO_STREAM("Dowsampling cur_cloud.");
    XYZPointCloud cur_down_cloud = downsampleCloud(cur_cloud);
    ROS_INFO_STREAM("Differencing clouds.");
    ProtoObjects moved = pointCloudDifference(prev_objs_cloud_,
                                              cur_down_cloud);
    // TODO: Get object differences
    ROS_INFO_STREAM("Updating previous cloud.");
    updatePreviousCloud(cur_down_cloud);
  }

 protected:
  XYZPointCloud cur_plane_cloud_;
  XYZPointCloud cur_objs_cloud_;
  XYZPointCloud prev_objs_cloud_;
  bool have_cur_cloud_;
  FeatureTracker* ft_;
  Eigen::Vector4f table_centroid_;

 public:
  double min_table_z_;
  double max_table_z_;
  double min_workspace_x_;
  double max_workspace_x_;
  double table_ransac_thresh_;
  double cluster_tolerance_;
  double cloud_diff_thresh_;
  int min_cluster_size_;
  int max_cluster_size_;
  double norm_est_radius_;
  double voxel_down_res_;
  bool use_voxel_down_;
  ros::Publisher pcl_obj_seg_pub_;
};

class ObjectSingulation
{
 public:
  ObjectSingulation(FeatureTracker* ft,
                    int kmeans_max_iter=200, double kmeans_epsilon=0.5,
                    int kmeans_tries=5, int affine_estimate_radius=5,
                    double surf_hessian=100) :
      ft_(ft),
      kmeans_max_iter_(kmeans_max_iter), kmeans_epsilon_(kmeans_epsilon),
      kmeans_tries_(kmeans_tries),
      affine_estimate_radius_(affine_estimate_radius)
  {
    // Create derivative kernels for flow calculation
    cv::getDerivKernels(dy_kernel_, dx_kernel_, 1, 0, CV_SCHARR, true, CV_32F);
    cv::flip(dy_kernel_, dy_kernel_, -1);
    cv::transpose(dy_kernel_, dx_kernel_);
  }

  // TODO: trim down the parameters
  /**
   * Determine the pushing pose and direction to verify separate objects
   *
   * @param motion_mask 
   * @param arm_mask 
   * @param workspace_mask 
   * @param color_img 
   * @param depth_img 
   * @param u 
   * @param v 
   * @param objs 
   *
   * @return 
   */
  PoseStamped getPushVector(cv::Mat& motion_mask, cv::Mat& arm_mask,
                            cv::Mat& workspace_mask, cv::Mat& color_img,
                            cv::Mat& depth_img, cv::Mat& u, cv::Mat& v,
                            ProtoObjects objs)
  {
    PoseStamped push_dir;
    cv::Mat boundary_img = getObjectBoundaryStrengths(motion_mask,
                                                      workspace_mask,
                                                      color_img, depth_img);
    cv::Mat push_pose_img = determineImgPushPoses(boundary_img, objs, arm_mask);
    return determinePushVector(push_pose_img);
  }

  cv::Mat determineImgPushPoses(cv::Mat& boundary_img, ProtoObjects objs,
                                cv::Mat& arm_mask)
  {
    cv::Mat push_pose_img;
    // TODO: Project objs into an image
    // TODO: For each object determine internal boundaries above some threshold
    // as locations for boundaries to exist
    // TODO: Determine pushing locations related to these possible boundaries
    // TODO: Determine seperation direction in the image from boundary (up, down, left, right)
    return push_pose_img;
  }

  // TODO: Determine 3D push_pose given a image location and direction
  PoseStamped determinePushVector(cv::Mat push_pose_img)
  {
    PoseStamped push_pose;
    return push_pose;
  }

  cv::Mat getObjectBoundaryStrengths(cv::Mat& motion_mask,
                                     cv::Mat& workspace_mask,
                                     cv::Mat& color_img, cv::Mat& depth_img)
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
    tmp_bw.convertTo(bw_img, CV_32FC1, 1.0/255, 0);

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

    // TODO: Replace with a learned function from a combination of cues

    // Remove stuff out of the image
    edge_img.copyTo(edge_img_masked, workspace_mask);
    depth_edge_img.copyTo(depth_edge_img_masked, workspace_mask);
    cv::Mat combined_edges = cv::max(edge_img_masked, depth_edge_img_masked);

#ifdef DISPLAY_OBJECT_BOUNDARIES
    cv::imshow("boundary_strengths", edge_img_masked);
    cv::imshow("depth_boundary_strengths", depth_edge_img_masked);
    cv::imshow("combined_boundary_strengths", combined_edges);
#endif // DISPLAY_OBJECT_BOUNDARIES
    return edge_img;
  }

  //
  // Class member variables
  //
 protected:
  cv::Mat dx_kernel_;
  cv::Mat dy_kernel_;
  cv::Mat g_kernel_;
  FeatureTracker* ft_;
 public:
  int kmeans_max_iter_;
  double kmeans_epsilon_;
  double ransac_epsilon_;
  double ransac_inlier_percent_est_;
  int kmeans_tries_;
  int max_k_;
  int affine_estimate_radius_;
  int min_affine_point_set_size_;
  int max_ransac_iters_;
  double minimum_new_cluster_separation_;
};

class TabletopPushingPerceptionNode
{
 public:
  TabletopPushingPerceptionNode(ros::NodeHandle &n) :
      n_(n), n_private_("~"),
      image_sub_(n, "color_image_topic", 1),
      depth_sub_(n, "depth_image_topic", 1),
      cloud_sub_(n, "point_cloud_topic", 1),
      sync_(MySyncPolicy(15), image_sub_, depth_sub_, cloud_sub_),
      it_(n),
      singulation_server_(n, "singulation_action",
                    boost::bind(&TabletopPushingPerceptionNode::trackerGoalCallback,
                                this, _1),
                    false),
      tf_(), ft_("pushing_perception"), os_(&ft_), pcl_segmenter_(&ft_),
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
    n_private_.param("below_table_z", below_table_z_, 0.1);
    n_private_.param("max_workspace_x", max_workspace_x_, 0.0);
    n_private_.param("max_workspace_y", max_workspace_y_, 0.0);
    n_private_.param("max_workspace_z", max_workspace_z_, 0.0);
    n_private_.param("min_pushing_x", min_pushing_x_, 0.0);
    n_private_.param("min_pushing_y", min_pushing_y_, 0.0);
    n_private_.param("max_pushing_x", max_pushing_x_, 0.0);
    n_private_.param("max_pushing_y", max_pushing_y_, 0.0);
    std::string default_workspace_frame = "/torso_lift_link";
    n_private_.param("workspace_frame", workspace_frame_,
                    default_workspace_frame);

    n_private_.param("min_table_z", pcl_segmenter_.min_table_z_, -0.5);
    n_private_.param("max_table_z", pcl_segmenter_.max_table_z_, 1.5);
    pcl_segmenter_.min_workspace_x_ = min_workspace_x_;
    pcl_segmenter_.max_workspace_x_ = max_workspace_x_;

    n_private_.param("segmenting_moving_stuff", segmenting_moving_stuff_,
                     false);
    n_private_.param("autostart_tracking", tracking_, false);
    n_private_.param("auto_flow_cluster", auto_flow_cluster_, false);
    n_private_.param("autostart_pcl_segmentation", autorun_pcl_segmentation_,
                     false);

    n_private_.param("num_downsamples", num_downsamples_, 2);
    std::string cam_info_topic_def = "/kinect_head/rgb/camera_info";
    n_private_.param("cam_info_topic", cam_info_topic_,
                    cam_info_topic_def);
    n_private_.param("table_ransac_thresh", pcl_segmenter_.table_ransac_thresh_,
                     0.01);

    base_output_path_ = "/home/thermans/sandbox/cut_out/";
    // Graphcut weights
    // n_private_.param("mgc_workspace_bg_weight",
    //                 mgc_.workspace_background_weight_, 1.0);
    // n_private_.param("mgc_min_weight", mgc_.min_weight_, 0.01);
    // n_private_.param("mgc_w_c_alpha", mgc_.w_c_alpha_, 0.1);
    // n_private_.param("mgc_w_c_beta",  mgc_.w_c_beta_, 0.1);
    // n_private_.param("mgc_w_c_gamma", mgc_.w_c_gamma_, 0.1);
    // n_private_.param("mgc_arm_grow_radius", mgc_.arm_grow_radius_, 2);
    // n_private_.param("mgc_arm_search_radius", mgc_.arm_search_radius_, 50);
    // // Lucas Kanade params
    // n_private_.param("mgc_magnitude_thresh", mgc_.magnitude_thresh_, 0.1);
    // n_private_.param("mgc_flow_gain", mgc_.flow_gain_, 0.3);
    // n_private_.param("mgc_table_var", mgc_.table_height_var_, 0.03);
    // n_private_.param("mgc_arm_dist_var", mgc_.arm_dist_var_, 20.0);
    // n_private_.param("mgc_arm_color_var_add", mgc_.arm_color_var_add_, 0.1);
    // n_private_.param("mgc_arm_color_weight", mgc_.arm_alpha_, 0.5);
    // n_private_.param("mgc_arm_dist_weight", mgc_.arm_beta_, 0.5);
    int win_size = 5;
    n_private_.param("lk_win_size", win_size, 5);
    lkflow_.setWinSize(win_size);
    int num_levels = 4;
    n_private_.param("lk_num_levels", num_levels, 4);
    lkflow_.setNumLevels(num_levels);
    n_private_.param("lk_ratio_thresh", lkflow_.r_thresh_, 30.0);
    n_private_.param("max_flow_clusters", os_.max_k_, 2);
    n_private_.param("flow_cluster_max_iter", os_.kmeans_max_iter_, 200);
    n_private_.param("flow_cluster_epsilon", os_.kmeans_epsilon_, 0.05);
    n_private_.param("flow_cluster_attempts", os_.kmeans_tries_, 5);
    n_private_.param("affine_estimate_radius", os_.affine_estimate_radius_, 5);
    n_private_.param("min_affine_point_set_size", os_.min_affine_point_set_size_,
                     100);
    n_private_.param("max_ransac_iters", os_.max_ransac_iters_,
                     150);
    n_private_.param("affine_RANSAC_epsilon", os_.ransac_epsilon_, 1.0);
    n_private_.param("affine_RANSAC_inlier_percent",
                     os_.ransac_inlier_percent_est_, 0.05);
    n_private_.param("minimum_new_cluster_separation",
                     os_.minimum_new_cluster_separation_, 5.0);
    n_private_.param("surf_hessian_thresh", ft_.surf_.hessianThreshold,
                     150.0);
    bool use_fast;
    n_private_.param("use_fast_corners", use_fast, false);
    ft_.setUseFast(use_fast);
    n_private_.param("image_hist_size", image_hist_size_, 5);
    n_private_.param("pcl_cluster_tolerance", pcl_segmenter_.cluster_tolerance_,
                     0.25);
    n_private_.param("pcl_difference_thresh", pcl_segmenter_.cloud_diff_thresh_,
                     0.01);
    n_private_.param("pcl_min_cluster_size", pcl_segmenter_.min_cluster_size_,
                     100);
    n_private_.param("pcl_max_cluster_size", pcl_segmenter_.max_cluster_size_,
                     2500);
    n_private_.param("normal_estimate_search_radius", norm_est_radius_, 0.03);
    n_private_.param("pcl_voxel_downsample_res", pcl_segmenter_.voxel_down_res_,
                     0.005);
    n_private_.param("use_pcl_voxel_downsample", pcl_segmenter_.use_voxel_down_,
                     true);

    // Setup ros node connections
    sync_.registerCallback(&TabletopPushingPerceptionNode::sensorCallback,
                           this);
    push_pose_server_ = n_.advertiseService(
        "get_push_pose", &TabletopPushingPerceptionNode::getPushPose, this);
    table_location_server_ = n_.advertiseService(
        "get_table_location", &TabletopPushingPerceptionNode::getTableLocation,
        this);
    motion_mask_pub_ = it_.advertise("motion_mask", 15);
    motion_img_pub_ = it_.advertise("motion_img", 15);
    arm_mask_pub_ = it_.advertise("arm_mask", 15);
    arm_img_pub_ = it_.advertise("arm_img", 15);
    pcl_segmenter_.pcl_obj_seg_pub_ = n_.advertise<sensor_msgs::PointCloud2>(
        "separate_table_objs", 1000);
    singulation_server_.start();
  }

  void sensorCallback(const sensor_msgs::ImageConstPtr& img_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg,
                      const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
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

    // Debug stuff
    if (autorun_pcl_segmentation_) getPushVector();// findRandomPushPose(cloud);

    // Started via actionlib call
    updateTracks(cur_color_frame_, cur_depth_frame_, prev_color_frame_,
                 prev_depth_frame_, cur_point_cloud_);
    // if (segmenting_moving_stuff_)
    // {
    //   cv::Mat seg_mask = segmentMovingStuff(cur_color_frame_, cur_depth_frame_,
    //                                         prev_color_frame_, prev_depth_frame_,
    //                                         cur_point_cloud_);
    //   prev_seg_mask_ = seg_mask.clone();
    //   getPushVector();
    // }

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
#if defined DISPLAY_INPUT_COLOR || defined DISPLAY_INPUT_DEPTH || defined DISPLAY_OPTICAL_FLOW || defined DISPLAY_GRAPHCUT || defined DISPLAY_WORKSPACE_MASK || defined DISPLAY_OPT_FLOW_INTERNALS || defined DISPLAY_GRAPHCUT || defined VISUALIZE_GRAPH_WEIGHTS || defined VISUALIZE_ARM_GRAPH_WEIGHTS || defined DISPLAY_ARM_CIRCLES || defined DISPLAY_FLOW_FIELD_CLUSTERING
    cv::waitKey(display_wait_ms_);
#endif // Any display defined
  }

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

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    if ( have_depth_data_ )
    {
      res = findPushPose(cur_color_frame_, cur_depth_frame_, cur_point_cloud_,
                         req.use_guided);
    }
    else
    {
      ROS_ERROR_STREAM("Calling getPushPose prior to receiving sensor data.");
      return false;
    }
    return true;
  }

  PoseStamped getPushVector()
  {
    if (tracker_count_ == 1)
    {
      ROS_INFO_STREAM("Finding tabletop objects. tracker_count_ == 1");
      ProtoObjects objs = pcl_segmenter_.findTabletopObjects(cur_point_cloud_);
    }
    else if (tracker_count_ > 1)
    {
      ROS_INFO_STREAM("Updating tabletop objects. tracker_count_ > 1");
      // TODO: Store downsampled versions of these internal to pcl_segmenter
      // or object singulation...
      ProtoObjects objs = pcl_segmenter_.updateObjects(cur_point_cloud_);
      // TODO: trim down what gets sent here
      return os_.getPushVector(motion_mask_hist_.back(),
                               arm_mask_hist_.back(),
                               workspace_mask_hist_.back(),
                               color_frame_hist_.back(),
                               depth_frame_hist_.back(),
                               flow_u_hist_.back(),
                               flow_v_hist_.back(), objs);
    }
    PoseStamped empty;
    return empty;
  }

  PushPose::Response findPushPose(cv::Mat visual_frame,
                                  cv::Mat depth_frame,
                                  XYZPointCloud& cloud, bool use_guided)
  {
    PushPose::Response res;
    if (use_guided)
    {
      res.push_pose = getPushVector();
    }
    else
    {
      res.push_pose = findRandomPushPose(cloud);
    }
    res.invalid_push_pose = false;
    return res;
  }

  //
  // Region tracking methods
  //

  void trackerGoalCallback(
      const tabletop_pushing::ObjectSingulationGoalConstPtr &goal)
  {
    if (goal->init)
    {
      ROS_INFO_STREAM("Initializing tracker.");
      initTracker();
      tabletop_pushing::ObjectSingulationResult result;
      singulation_server_.setSucceeded(result);
    }

    if (goal->start)
    {
      ROS_INFO_STREAM("Starting tracker.");
      startTracker();
      tabletop_pushing::ObjectSingulationResult result;
      singulation_server_.setSucceeded(result);
    }
    else
    {
      ROS_INFO_STREAM("Stopping tracker.");
      stopTracker();
      tabletop_pushing::ObjectSingulationResult result;
      if (goal->get_singulation_vector)
      {
        result.singulation_vector = getPushVector();
      }
      singulation_server_.setSucceeded(result);
      // singulation_server_.setPreempted();
    }
  }

  void initTracker()
  {
    tracker_initialized_ = false;
    tracking_ = false;
  }

  void startTracker()
  {
    // tracker_initialized_ = true;
    tracking_ = true;
  }

  void stopTracker()
  {
    // tracker_initialized_ = false;
    tracking_ = false;
  }

  void updateTracks(cv::Mat color_frame, cv::Mat& depth_frame,
                    cv::Mat& prev_color_frame, cv::Mat& prev_depth_frame,
                    XYZPointCloud& cloud)
  {
    // Get a grayscale image as well
    cv::Mat gray_frame;
    cv::cvtColor(color_frame, gray_frame, CV_BGR2GRAY);

    if (!tracker_initialized_)
    {
      cam_info_ = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
          cam_info_topic_, n_, ros::Duration(5.0));

      table_centroid_ = getTablePlane(cloud);
      if (table_centroid_.pose.position.x == 0.0 &&
          table_centroid_.pose.position.y == 0.0 &&
          table_centroid_.pose.position.z == 0.0)
      {
        ROS_DEBUG_STREAM("No plane found!");
      }
      else
      {
        min_workspace_z_ = table_centroid_.pose.position.z - below_table_z_;
        n_private_.setParam("min_workspace_z", min_workspace_z_);
        ROS_DEBUG_STREAM("Found plane");
      }

      ft_.initTracks(gray_frame);

      tracker_initialized_ = true;
      tracker_count_ = 0;
      return;
    }
    if (!tracking_)
    {
      return;
    }
    // Get sparse flow
    AffineFlowMeasures sparse_flow = ft_.updateTracks(gray_frame,
                                                      cur_workspace_mask_);
    ++tracker_count_;
  }

  //
  // Core method for calculation
  //
//   cv::Mat segmentMovingStuff(cv::Mat& color_frame, cv::Mat& depth_frame,
//                              cv::Mat& prev_color_frame,
//                              cv::Mat& prev_depth_frame, XYZPointCloud& cloud)
//   {
//     if (!tracking_ || !tracker_initialized_ || tracker_count_ < 1)
//     {
//       cv::Mat empty_segments(color_frame.rows, color_frame.cols, CV_8UC1,
//                              cv::Scalar(0));
//       return empty_segments;
//     }

//     // TODO: Consolidate into a single function call ?
//     // Convert frame to floating point HSV
//     cv::Mat color_frame_hsv(color_frame.size(), color_frame.type());
//     cv::cvtColor(color_frame, color_frame_hsv, CV_BGR2HSV);
//     cv::Mat color_frame_f(color_frame_hsv.size(), CV_32FC3);
//     color_frame_hsv.convertTo(color_frame_f, CV_32FC3, 1.0/255, 0);

//     // Get optical flow
//     std::vector<cv::Mat> flow_outs = lkflow_(color_frame, prev_color_frame);

//     // Project locations of the arms and hands into the image
//     int min_arm_x = 0;
//     int max_arm_x = 0;
//     int min_arm_y = 0;
//     int max_arm_y = 0;
//     ArmModel hands_and_arms = projectArmPoses(cur_camera_header_,
//                                               color_frame.size(), min_arm_x,
//                                               max_arm_x, min_arm_y, max_arm_y);
//     // Get pixel heights above the table
//     cv::Mat heights_above_table = getTableHeightDistances();
//     // Perform graphcut for motion detection
//     cv::Mat cut = mgc_(color_frame_f, depth_frame, flow_outs[0], flow_outs[1],
//                        cur_workspace_mask_, heights_above_table,
//                        hands_and_arms);
//     // Perform graphcut for arm localization
//     cv::Mat arm_cut(color_frame.size(), CV_8UC1, cv::Scalar(0));
//     if (hands_and_arms[0].size() > 0 || hands_and_arms[1].size() > 0)
//     {
//       arm_cut = mgc_.segmentRobotArm(color_frame_f, depth_frame,
//                                      cur_workspace_mask_, hands_and_arms,
//                                      min_arm_x, max_arm_x, min_arm_y, max_arm_y);
//     }

//     // Publish the moving region stuff
//     cv_bridge::CvImage motion_mask_msg;
//     motion_mask_msg.image = cut;
//     motion_mask_msg.header = cur_camera_header_;
//     motion_mask_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
//     motion_mask_pub_.publish(motion_mask_msg.toImageMsg());

//     // Publish arm stuff
//     cv_bridge::CvImage arm_mask_msg;
//     arm_mask_msg.image = arm_cut;
//     arm_mask_msg.header = cur_camera_header_;
//     arm_mask_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
//     arm_mask_pub_.publish(arm_mask_msg.toImageMsg());

//     // Also publish color versions
//     cv::Mat moving_regions_img;
//     color_frame.copyTo(moving_regions_img, cut);
//     cv_bridge::CvImage motion_img_msg;
//     cv::Mat motion_img_send(cut.size(), CV_8UC3);
//     moving_regions_img.convertTo(motion_img_send, CV_8UC3, 1.0, 0);
//     motion_img_msg.image = motion_img_send;
//     motion_img_msg.header = cur_camera_header_;
//     motion_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
//     motion_img_pub_.publish(motion_img_msg.toImageMsg());

//     cv::Mat arm_regions_img;
//     color_frame.copyTo(arm_regions_img, arm_cut);
//     cv_bridge::CvImage arm_img_msg;
//     cv::Mat arm_img_send(arm_regions_img.size(), CV_8UC3);
//     arm_regions_img.convertTo(arm_img_send, CV_8UC3, 1.0, 0);
//     arm_img_msg.image = arm_img_send;
//     arm_img_msg.header = cur_camera_header_;
//     arm_img_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
//     arm_img_pub_.publish(arm_img_msg.toImageMsg());
//     // cv::Mat not_arm_move = cut - arm_cut;
//     // cv::Mat not_arm_move_color;
//     // color_frame.copyTo(not_arm_move_color, not_arm_move);

//     // Get point cloud associateds with the motion mask and arm mask
//     // XYZPointCloud moving_cloud = getMaskedPointCloud(cloud, cut);
//     // XYZPointCloud arm_cloud = getMaskedPointCloud(cloud, arm_cut);

//     cv::Mat last_motion_mask;
//     cv::Mat last_arm_mask;
//     cv::Mat last_workspace_mask;
//     cv::Mat last_color_frame;
//     cv::Mat last_depth_frame;
//     cv::Mat last_flow_u;
//     cv::Mat last_flow_v;

//     cut.copyTo(last_motion_mask);
//     arm_cut.copyTo(last_arm_mask);
//     cur_workspace_mask_.copyTo(last_workspace_mask);
//     color_frame.copyTo(last_color_frame);
//     depth_frame.copyTo(last_depth_frame);
//     flow_outs[0].copyTo(last_flow_u);
//     flow_outs[1].copyTo(last_flow_v);

//     motion_mask_hist_.push_back(last_motion_mask);
//     arm_mask_hist_.push_back(last_arm_mask);
//     workspace_mask_hist_.push_back(last_workspace_mask);
//     color_frame_hist_.push_back(last_color_frame);
//     depth_frame_hist_.push_back(last_depth_frame);
//     flow_u_hist_.push_back(last_flow_u);
//     flow_v_hist_.push_back(last_flow_v);

//     if (motion_mask_hist_.size() > image_hist_size_)
//     {
//       motion_mask_hist_.pop_front();
//       arm_mask_hist_.pop_front();
//       workspace_mask_hist_.pop_front();
//       color_frame_hist_.pop_front();
//       depth_frame_hist_.pop_front();
//       flow_u_hist_.pop_front();
//       flow_v_hist_.pop_front();
//     }

// #ifdef WRITE_INPUT_TO_DISK
//     std::stringstream input_out_name;
//     input_out_name << base_output_path_ << "input" << tracker_count_ << ".tiff";
//     cv::imwrite(input_out_name.str(), color_frame);
// #endif // WRITE_INPUT_TO_DISK
// #ifdef WRITE_FLOWS_TO_DISK
//     cv::Mat flow_thresh_disp_img(color_frame.size(), CV_8UC3);
//     flow_thresh_disp_img = color_frame.clone();
//     for (int r = 0; r < flow_thresh_disp_img.rows; ++r)
//     {
//       for (int c = 0; c < flow_thresh_disp_img.cols; ++c)
//       {
//         float u = flow_outs[0].at<float>(r,c);
//         float v = flow_outs[1].at<float>(r,c);
//         if (std::sqrt(u*u+v*v) > mgc_.magnitude_thresh_)
//         {
//           cv::line(flow_thresh_disp_img, cv::Point(c,r),
//                    cv::Point(c-u, r-v), cv::Scalar(0,255,0));
//         }
//       }
//     }

//     std::stringstream flow_out_name;
//     flow_out_name << base_output_path_ << "flow" << tracker_count_ << ".tiff";
//     cv::imwrite(flow_out_name.str(), flow_thresh_disp_img);
// #endif // WRITE_FLOWS_TO_DISK
// #ifdef WRITE_CUTS_TO_DISK
//     std::stringstream cut_out_name;
//     cut_out_name << base_output_path_ << "cut" << tracker_count_ << ".tiff";
//     cv::imwrite(cut_out_name.str(), moving_regions_img);
// #endif // WRITE_CUTS_TO_DISK
// #ifdef WRITE_ARM_CUT_TO_DISK
//     std::stringstream arm_cut_out_name;
//     arm_cut_out_name << base_output_path_ << "arm_cut" << tracker_count_ << ".tiff";
//     cv::imwrite(arm_cut_out_name.str(), arm_regions_img);
//     // std::stringstream not_arm_move_out_name;
//     // not_arm_move_out_name << base_output_path_ << "not_arm_move" << tracker_count_
//     //                      << ".tiff";
//     // cv::imwrite(not_arm_move_out_name.str(), not_arm_move_color);
// #endif // WRITE_ARM_CUT_TO_DISK
// #ifdef DISPLAY_OPTICAL_FLOW
//     cpl_visual_features::displayOpticalFlow(color_frame, flow_outs[0],
//                                             flow_outs[1],
//                                             mgc_.magnitude_thresh_);
// #endif // DISPLAY_OPTICAL_FLOW
// #ifdef DISPLAY_ARM_CIRCLES
//     cv::Mat arms_img(color_frame.size(), CV_8UC3);
//     arms_img = color_frame.clone();
//     for (unsigned int i = 0; i < hands_and_arms.size(); ++i)
//     {
//       for (unsigned int j = 0; j < hands_and_arms[i].size(); ++j)
//       {
//         cv::Scalar color;
//         if (i%2 == 0)
//         {
//           color = cv::Scalar(0,0,255);
//         }
//         else
//         {
//           color = cv::Scalar(0,255,0);
//         }
//         cv::circle(arms_img, hands_and_arms[i][j], 2, color);
//       }
//     }
//     cv::imshow("arms", arms_img);
// #endif
// #ifdef DISPLAY_GRAPHCUT
//     cv::imshow("moving_regions", moving_regions_img);
//     cv::imshow("arm_cut", arm_regions_img);
//     // cv::imshow("not_arm_move", not_arm_move_color);
// #endif // DISPLAY_GRAPHCUT
//     return cut;
//   }

  //
  // Arm detection methods
  //
  cv::Point projectPointIntoImage(PointStamped cur_point,
                                  std::string target_frame)
  {
    cv::Point img_loc;
    try
    {
      // Transform point into the camera frame
      PointStamped image_frame_loc_m;
      tf_.transformPoint(target_frame, cur_point, image_frame_loc_m);
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

  cv::Point projectPointIntoImage(PoseStamped cur_pose,
                                  std::string target_frame)
  {
    PointStamped cur_point;
    cur_point.header = cur_pose.header;
    cur_point.point = cur_pose.pose.position;
    return projectPointIntoImage(cur_point, target_frame);
  }

  bool getLineValues(cv::Point p1, cv::Point p2, std::vector<cv::Point>& line,
                     cv::Size frame_size,
                     int &min_x, int &max_x, int &min_y, int &max_y)
  {
    int num_points_added = 0;
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
          if (p_new.x < min_x)
            min_x = p_new.x;
          if (p_new.x > max_x)
            max_x = p_new.x;
          if (p_new.y < min_y)
            min_y = p_new.y;
          if (p_new.y > max_y)
            max_y = p_new.y;
          line.push_back(p_new);
          ++num_points_added;
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
          if (p_new.x < min_x)
            min_x = p_new.x;
          if (p_new.x > max_x)
            max_x = p_new.x;
          if (p_new.y < min_y)
            min_y = p_new.y;
          if (p_new.y > max_y)
            max_y = p_new.y;
          line.push_back(p_new);
          ++num_points_added;
        }
      }
      error -= dy;
      if (error < 0)
      {
        y += ystep;
        error += dx;
      }
    }
    return (num_points_added > 0);
  }

  ArmModel projectArmPoses(std_msgs::Header img_header, cv::Size frame_size,
                           int &min_x, int &max_x, int &min_y, int &max_y)
  {
    // Project all arm joints into image
    ArmModel arm_locs;

    // Left hand
    cv::Point ll0 = projectJointOriginIntoImage(img_header,
                                                "l_gripper_l_finger_tip_link");
    cv::Point ll1 = projectJointOriginIntoImage(img_header,
                                                "l_gripper_l_finger_link");
    cv::Point lr0 = projectJointOriginIntoImage(img_header,
                                                "l_gripper_r_finger_tip_link");
    cv::Point lr1 = projectJointOriginIntoImage(img_header,
                                                "l_gripper_r_finger_link");
    cv::Point lp = projectJointOriginIntoImage(img_header,
                                               "l_gripper_palm_link");
    // TODO: Add more arm locations
    // Left arm
    cv::Point l1 = projectJointOriginIntoImage(img_header, "l_wrist_flex_link");
    cv::Point l2 = projectJointOriginIntoImage(img_header, "l_forearm_link");
    cv::Point l3 = projectJointOriginIntoImage(img_header, "l_upper_arm_link");
    arm_locs.l_chain.push_back(lp);
    arm_locs.l_chain.push_back(ll1);
    arm_locs.l_chain.push_back(ll0);
    arm_locs.l_chain.push_back(lr0);
    arm_locs.l_chain.push_back(lr1);
    arm_locs.l_chain.push_back(lp);
    arm_locs.l_chain.push_back(l1);
    arm_locs.l_chain.push_back(l2);
    arm_locs.l_chain.push_back(l3);

    // Right hand
    cv::Point rl0 = projectJointOriginIntoImage(img_header,
                                                "r_gripper_l_finger_tip_link");
    cv::Point rl1 = projectJointOriginIntoImage(img_header,
                                                "r_gripper_l_finger_link");
    cv::Point rr0 = projectJointOriginIntoImage(img_header,
                                                "r_gripper_r_finger_tip_link");
    cv::Point rr1 = projectJointOriginIntoImage(img_header,
                                                "r_gripper_r_finger_link");
    cv::Point rp = projectJointOriginIntoImage(img_header,
                                               "r_gripper_palm_link");

    // TODO: Add more arm locations
    // Right arm
    cv::Point r1 = projectJointOriginIntoImage(img_header, "r_wrist_flex_link");
    cv::Point r2 = projectJointOriginIntoImage(img_header, "r_forearm_link");
    cv::Point r3 = projectJointOriginIntoImage(img_header, "r_upper_arm_link");
    arm_locs.r_chain.push_back(rp);
    arm_locs.r_chain.push_back(rl1);
    arm_locs.r_chain.push_back(rl0);
    arm_locs.r_chain.push_back(rr0);
    arm_locs.r_chain.push_back(rr1);
    arm_locs.r_chain.push_back(rp);
    arm_locs.r_chain.push_back(r1);
    arm_locs.r_chain.push_back(r2);
    arm_locs.r_chain.push_back(r3);

    // Keep track of min and max values
    min_x = 10000;
    max_x = 0;
    min_y = 10000;
    max_y = 0;

    // Add left hand
    arm_locs.l_hand_on = getLineValues(ll0, ll1, arm_locs.hands, frame_size,
                                       min_x, max_x, min_y, max_y);
    arm_locs.l_hand_on = (getLineValues(ll1, lp, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                          arm_locs.l_hand_on);
    arm_locs.l_hand_on = (getLineValues(lr0, lr1, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                          arm_locs.l_hand_on);
    arm_locs.l_hand_on = (getLineValues(lr1, lp, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                          arm_locs.l_hand_on);
    // Add left arm
    arm_locs.l_arm_on = getLineValues(lp, l1, arm_locs.arms, frame_size,
                                      min_x, max_x, min_y, max_y);
    arm_locs.l_arm_on = (getLineValues(l1, l2, arm_locs.arms, frame_size,
                                      min_x, max_x, min_y, max_y) ||
                         arm_locs.l_arm_on);
    arm_locs.l_arm_on = (getLineValues(l2, l3, arm_locs.arms, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.l_arm_on);
    // Add right hand
    arm_locs.r_hand_on = (getLineValues(rl0, rl1, arm_locs.hands, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.r_hand_on);
    arm_locs.r_hand_on = (getLineValues(rl1, rp, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                         arm_locs.r_hand_on);
    arm_locs.r_hand_on = (getLineValues(rr0, rr1, arm_locs.hands, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.r_hand_on);
    arm_locs.r_hand_on = (getLineValues(rr1, rp, arm_locs.hands, frame_size,
                                        min_x, max_x, min_y, max_y) ||
                         arm_locs.r_hand_on);
    // Add right arm
    arm_locs.r_arm_on = getLineValues(rp, r1, arm_locs.arms, frame_size,
                                      min_x, max_x, min_y, max_y);
    arm_locs.r_arm_on = (getLineValues(r1, r2, arm_locs.arms, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.r_arm_on);
    arm_locs.r_arm_on = (getLineValues(r2, r3, arm_locs.arms, frame_size,
                                       min_x, max_x, min_y, max_y) ||
                         arm_locs.r_arm_on);
    return arm_locs;
  }

  cv::Point projectJointOriginIntoImage(std_msgs::Header img_header,
                                        std::string joint_name)
  {
    PointStamped joint_origin;
    joint_origin.header.frame_id = joint_name;
    joint_origin.header.stamp = img_header.stamp;
    joint_origin.point.x = 0.0;
    joint_origin.point.y = 0.0;
    joint_origin.point.z = 0.0;
    // TODO: Use more information in setting correct locations
    if (joint_name == "r_gripper_tool_frame" ||
        joint_name == "l_gripper_tool_frame" ||
        joint_name == "l_gripper_l_finger_tip_link" ||
        joint_name == "l_gripper_l_finger_link" ||
        joint_name == "l_gripper_r_finger_tip_link" ||
        joint_name == "l_gripper_r_finger_link" ||
        joint_name == "r_gripper_l_finger_tip_link" ||
        joint_name == "r_gripper_l_finger_link" ||
        joint_name == "r_gripper_r_finger_tip_link" ||
        joint_name == "r_gripper_r_finger_link" ||
        joint_name == "r_gripper_tool_frame")
    {
      joint_origin.point.z = 0.01;
      if (joint_name == "r_gripper_l_finger_link" ||
          joint_name == "l_gripper_l_finger_link")
      {
        joint_origin.point.y = 0.02;
      }
      else if (joint_name == "r_gripper_r_finger_link" ||
               joint_name == "l_gripper_r_finger_link")
      {
        joint_origin.point.y = -0.02;
      }
    }
    // TODO: Check this better
    if (joint_name == "r_upper_arm_link")
    {
      joint_origin.point.y = -0.05;
    }
    // TODO: Check this better
    if (joint_name == "l_upper_arm_link")
    {
      joint_origin.point.y = 0.05;
    }
    if (joint_name == "r_gripper_palm_link")
    {
      joint_origin.point.x = 0.05;
    }
    if (joint_name == "l_gripper_palm_link")
    {
      joint_origin.point.x = 0.05;
    }
    return projectPointIntoImage(joint_origin, img_header.frame_id);
  }

  void drawTablePlaneOnImage(XYZPointCloud& plane, PoseStamped cent)
  {
    cv::Mat plane_display = cur_color_frame_.clone();
    std::vector<cv::Point> table_points;
    for (unsigned int i = 0; i < plane.points.size(); ++i)
    {
      PointStamped h;
      h.header = plane.header;
      h.point.x = plane.points[i].x;
      h.point.y = plane.points[i].y;
      h.point.z = plane.points[i].z;
      cv::Point p = projectPointIntoImage(h, cur_camera_header_.frame_id);
      table_points.push_back(p);
#ifdef DISPLAY_PLANE_ESTIMATE
      cv::circle(plane_display, p, 2, cv::Scalar(0,255,0));
#endif // DISPLAY_PLANE_ESTIMATE
    }

#ifdef USE_TABLE_COLOR_ESTIMATE
    ROS_INFO_STREAM("Calculating table color stats.");
    mgc_.setTableColorStats(cur_color_frame_, table_points);
#endif // USE_TABLE_COLOR_ESTIMATE

#ifdef DISPLAY_PLANE_ESTIMATE
    cv::Point cent_img = projectPointIntoImage(cent,
                                               cur_camera_header_.frame_id);
    cv::circle(plane_display, cent_img, 5, cv::Scalar(0,0,255));
    cv::imshow("table_image", plane_display);
#endif // DISPLAY_PLANE_ESTIMATE
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

  cv::Mat getTableHeightDistances()
  {
    cv::Mat table_distances(cur_depth_frame_.size(), CV_32FC1, cv::Scalar(0.0));
    const float table_height = table_centroid_.pose.position.z;
    for (int r = 0; r < table_distances.rows; ++r)
    {
      for (int c = 0; c < table_distances.cols; ++c)
      {
        pcl::PointXYZ cur_pt = cur_point_cloud_.at(2*c, 2*r);
        if (isnan(cur_pt.x) || isnan(cur_pt.y) || isnan(cur_pt.z))
        {
          // 3 meters is sufficiently far away
          table_distances.at<float>(r,c) = 3.0;
        }
        else
        {
          table_distances.at<float>(r,c) = cur_pt.z - table_height;
        }
      }
    }
#ifdef DISPLAY_TABLE_DISTANCES
    cv::imshow("table_distances_raw", table_distances);
#endif // DISPLAY_TABLE_DISTANCES
    return table_distances;
  }

  PoseStamped getTablePlane(XYZPointCloud& cloud)
  {
    Eigen::Vector4f table_centroid = pcl_segmenter_.getTablePlane(cloud);
    PoseStamped p;
    p.pose.position.x = table_centroid[0];
    p.pose.position.y = table_centroid[1];
    p.pose.position.z = table_centroid[2];
    p.header = cloud.header;
    ROS_INFO_STREAM("Table centroid is: ("
                    << p.pose.position.x << ", "
                    << p.pose.position.y << ", "
                    << p.pose.position.z << ")");
    // TODO: XYZPointCloud plane_cloud = pcl_segmenter_.getCurrentTablePoints()
    // drawTablePlaneOnImage(plane_cloud, p);
    return p;
  }

  PoseStamped findRandomPushPose(XYZPointCloud& input_cloud)
  {
    ProtoObjects objs = pcl_segmenter_.findTabletopObjects(input_cloud);
    prev_proto_objs_ = cur_proto_objs_;
    cur_proto_objs_ = objs;

    ROS_INFO_STREAM("Found " << objs.size() << " objects.");

    // TODO: publish a ros point cloud here for visualization
    // TODO: Move the publisher out of the segmentation class

    std::vector<Eigen::Vector4f> cluster_centroids;
    for (unsigned int i = 0; i < objs.size(); ++i)
    {
      if (objs[i].centroid[0] > min_pushing_x_ &&
          objs[i].centroid[0] < max_pushing_x_ &&
          objs[i].centroid[1] > min_pushing_y_ &&
          objs[i].centroid[1] < max_pushing_y_)
      {
        cluster_centroids.push_back(objs[i].centroid);
      }
    }
    geometry_msgs::PoseStamped p;

    if (cluster_centroids.size() < 1)
    {
      ROS_WARN_STREAM("No object clusters found! Returning empty push_pose");
      p.header.frame_id = "/torso_lift_link";
      return p;
    }
    ROS_INFO_STREAM("Found " << cluster_centroids.size() << " proto objects");
    int rand_idx = rand() % cluster_centroids.size();
    Eigen::Vector4f obj_xyz_centroid = cluster_centroids[rand_idx];
    p.pose.position.x = obj_xyz_centroid[0];
    p.pose.position.y = obj_xyz_centroid[1];
    // Set z to be the table height
    p.pose.position.z = objs[0].table_centroid[2];
    ROS_INFO_STREAM("Chosen push pose is at: (" << obj_xyz_centroid[0] << ", "
                    << obj_xyz_centroid[1] << ", " << objs[0].table_centroid[2]
                    << ")");

    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 0;

    p.header.frame_id = "/torso_lift_link";
    return p;
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
  image_transport::Publisher motion_img_pub_;
  image_transport::Publisher motion_mask_pub_;
  image_transport::Publisher arm_img_pub_;
  image_transport::Publisher arm_mask_pub_;
  actionlib::SimpleActionServer<tabletop_pushing::
                                ObjectSingulationAction> singulation_server_;
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
  cv::Mat prev_seg_mask_;
  std::deque<cv::Mat> motion_mask_hist_;
  std::deque<cv::Mat> arm_mask_hist_;
  std::deque<cv::Mat> workspace_mask_hist_;
  std::deque<cv::Mat> color_frame_hist_;
  std::deque<cv::Mat> depth_frame_hist_;
  std::deque<cv::Mat> flow_u_hist_;
  std::deque<cv::Mat> flow_v_hist_;
  std_msgs::Header cur_camera_header_;
  std_msgs::Header prev_camera_header_;
  XYZPointCloud cur_point_cloud_;
  DenseLKFlow lkflow_;
  FeatureTracker ft_;
  // MotionGraphcut mgc_;
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
  double min_pushing_x_;
  double max_pushing_x_;
  double min_pushing_y_;
  double max_pushing_y_;
  double below_table_z_;
  int num_downsamples_;
  std::string workspace_frame_;
  PoseStamped table_centroid_;
  bool tracking_;
  bool tracker_initialized_;
  std::string cam_info_topic_;
  int tracker_count_;
  std::string base_output_path_;
  int image_hist_size_;
  double norm_est_radius_;
  bool autorun_pcl_segmentation_;
  bool auto_flow_cluster_;
  bool segmenting_moving_stuff_;
  ProtoObjects prev_proto_objs_;
  ProtoObjects cur_proto_objs_;
};

int main(int argc, char ** argv)
{
  srand(time(NULL));
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  TabletopPushingPerceptionNode perception_node(n);
  perception_node.spin();

  return 0;
}
