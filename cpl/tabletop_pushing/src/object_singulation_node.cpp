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
// #define DISPLAY_PLANE_ESTIMATE 1
// #define DISPLAY_TABLE_DISTANCES 1
#define DISPLAY_OBJECT_BOUNDARIES 1
#define DISPLAY_WAIT 1

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

using cpl_visual_features::AffineFlowMeasure;
using cpl_visual_features::AffineFlowMeasures;
using cpl_visual_features::FeatureTracker;
using cpl_visual_features::Descriptor;

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
  PointCloudSegmentation(FeatureTracker* ft) : ft_(ft)
  {
  }

  /**
   * Function to determine the table plane in a point cloud
   *
   * @param cloud The cloud with the table as dominant plane.
   *
   * @return The centroid of the points belonging to the table plane.
   */
  Eigen::Vector4f getTablePlane(XYZPointCloud& cloud, XYZPointCloud& objs_cloud,
                                XYZPointCloud& plane_cloud)
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
    pcl::copyPointCloud(cloud_filtered, plane_inliers, plane_cloud);
    // Extract the outliers from the point clouds
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    XYZPointCloud objects_cloud;
    pcl::PointIndices plane_outliers;
    extract.setInputCloud(
        boost::make_shared<XYZPointCloud > (cloud_filtered));
    extract.setIndices(boost::make_shared<pcl::PointIndices> (plane_inliers));
    extract.setNegative(true);
    extract.filter(objs_cloud);
    // Extract the plane members into their own point cloud
    Eigen::Vector4f table_centroid;
    pcl::compute3DCentroid(plane_cloud, table_centroid);
    return table_centroid;
  }

  ProtoObjects findTabletopObjects(XYZPointCloud& input_cloud,
                                   bool extract_table=true)
  {
    XYZPointCloud objs_cloud;
    return findTabletopObjects(input_cloud, objs_cloud, extract_table);
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
                                   XYZPointCloud& objs_cloud,
                                   bool extract_table=true)
  {
    // XYZPointCloud objs_cloud;
    XYZPointCloud plane_cloud;
    // Get table plane
    if (extract_table)
    {
      table_centroid_ = getTablePlane(input_cloud, objs_cloud, plane_cloud);
      min_workspace_z_ = table_centroid_[2];
    }
    else
    {
      // TODO: Deal with not having to extract the tabletop each time
    }

    XYZPointCloud objects_cloud_down = downsampleCloud(objs_cloud);

    // Find independent regions
    ProtoObjects objs = clusterProtoObjects(objects_cloud_down, false);
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
      ROS_INFO_STREAM("Published label cloud.");
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

  /**
   * Get the objects that have moved between two point clouds
   *
   * @param prev_cloud The first cloud to use in differencing
   * @param cur_cloud The second cloud to use
   *
   * @return The new set of objects that have moved in the second cloud
   */
  ProtoObjects getMovedObjects(XYZPointCloud& prev_cloud,
                               XYZPointCloud& cur_cloud)
  {
    pcl::SegmentDifferences<pcl::PointXYZ> pcl_diff;
    pcl_diff.setDistanceThreshold(cloud_diff_thresh_);
    pcl_diff.setInputCloud(boost::make_shared<XYZPointCloud>(prev_cloud));
    pcl_diff.setTargetCloud(boost::make_shared<XYZPointCloud>(cur_cloud));
    XYZPointCloud cloud_out;
    pcl_diff.segment(cloud_out);
    ProtoObjects moved = clusterProtoObjects(cloud_out, true);
    // TODO: Get the tabletop objects from the current cloud and match them to
    // the moved objects
    return moved;
  }

  XYZPointCloud downsampleCloud(XYZPointCloud& cloud_in, bool pub_cloud=false)
  {
    XYZPointCloud cloud_z_filtered, cloud_x_filtered, cloud_down;
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setFilterFieldName("z");
    ROS_DEBUG_STREAM("Number of points in cloud_in is: " <<
                     cloud_in.size());
    z_pass.setInputCloud(boost::make_shared<XYZPointCloud>(cloud_in));
    z_pass.setFilterLimits(min_workspace_z_, max_workspace_z_);
    z_pass.filter(cloud_z_filtered);
    ROS_DEBUG_STREAM("Number of points in cloud_z_filtered is: " <<
                     cloud_z_filtered.size());

    // TODO: Filter in x?
    pcl::PassThrough<pcl::PointXYZ> x_pass;
    x_pass.setInputCloud(
        boost::make_shared<XYZPointCloud >(cloud_z_filtered));
    x_pass.setFilterFieldName("x");
    x_pass.setFilterLimits(min_workspace_x_, max_workspace_x_);
    x_pass.filter(cloud_x_filtered);

    pcl::VoxelGrid<pcl::PointXYZ> downsample_outliers;
    downsample_outliers.setInputCloud(
        boost::make_shared<XYZPointCloud>(cloud_x_filtered));
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

 protected:
  FeatureTracker* ft_;
  Eigen::Vector4f table_centroid_;

 public:
  double min_table_z_;
  double max_table_z_;
  double min_workspace_x_;
  double max_workspace_x_;
  double min_workspace_z_;
  double max_workspace_z_;
  double table_ransac_thresh_;
  double cluster_tolerance_;
  double cloud_diff_thresh_;
  int min_cluster_size_;
  int max_cluster_size_;
  double voxel_down_res_;
  bool use_voxel_down_;
  ros::Publisher pcl_obj_seg_pub_;
  ros::Publisher pcl_down_pub_;
};

class ObjectSingulation
{
 public:
  ObjectSingulation(FeatureTracker* ft, PointCloudSegmentation* pcl_segmenter) :
      ft_(ft), pcl_segmenter_(pcl_segmenter), callback_count_(0)
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
  PoseStamped getPushVector(cv::Mat& color_img, cv::Mat& depth_img,
                            XYZPointCloud& cloud, cv::Mat& workspace_mask)
  {
    ProtoObjects objs = calcProtoObjects(cloud);
    PoseStamped push_dir;
    cv::Mat boundary_img = getObjectBoundaryStrengths(color_img, depth_img,
                                                      workspace_mask);
    cv::Mat push_pose_img = determineImgPushPoses(boundary_img, objs);
    PoseStamped push_vector = determinePushVector(push_pose_img);
    // ROS_INFO_STREAM("Callback count: " << callback_count_);
    ++callback_count_;
    return push_vector;
  }

  PoseStamped findRandomPushPose(XYZPointCloud& input_cloud)
  {
    ProtoObjects objs = pcl_segmenter_->findTabletopObjects(input_cloud);
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

 protected:
  ProtoObjects calcProtoObjects(XYZPointCloud& cloud)
  {
    XYZPointCloud objs_cloud;
    ProtoObjects objs = pcl_segmenter_->findTabletopObjects(cloud, objs_cloud);
    XYZPointCloud cur_objs_down = pcl_segmenter_->downsampleCloud(objs_cloud,
                                                                  true);
    if (callback_count_ > 1)
    {
      ProtoObjects moved = pcl_segmenter_->getMovedObjects(prev_objs_down_,
                                                           cur_objs_down);
      // TODO: Get differences in the current protoobjects not just the moved
      // regions
      ROS_INFO_STREAM("Found " << moved.size() << " moved object regions.");
      objs = moved;
    }
    prev_objs_down_ = cur_objs_down;
    return objs;
  }

  cv::Mat determineImgPushPoses(cv::Mat& boundary_img, ProtoObjects objs)
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

  cv::Mat getObjectBoundaryStrengths(cv::Mat& color_img, cv::Mat& depth_img,
                                     cv::Mat& workspace_mask)
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
  PointCloudSegmentation* pcl_segmenter_;
  XYZPointCloud prev_cloud_down_;
  XYZPointCloud prev_objs_down_;
  ProtoObjects prev_proto_objs_;
  ProtoObjects cur_proto_objs_;
  int callback_count_;

 public:
  double min_pushing_x_;
  double max_pushing_x_;
  double min_pushing_y_;
  double max_pushing_y_;
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
      pcl_segmenter_(&ft_),
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
    std::string cam_info_topic_def = "/kinect_head/rgb/camera_info";
    n_private_.param("cam_info_topic", cam_info_topic_,
                    cam_info_topic_def);
    n_private_.param("table_ransac_thresh", pcl_segmenter_.table_ransac_thresh_,
                     0.01);

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
    n_private_.param("use_pcl_voxel_downsample", pcl_segmenter_.use_voxel_down_,
                     true);

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
    if (autorun_pcl_segmentation_) getPushPose();

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

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    if ( have_depth_data_ )
    {
      res.push_pose = getPushPose(req.use_guided);
      res.invalid_push_pose = false;
    }
    else
    {
      ROS_ERROR_STREAM("Calling getPushPose prior to receiving sensor data.");
      res.invalid_push_pose = true;
      return false;
    }
    return true;
  }

  PoseStamped getPushPose(bool use_guided=true)
  {
    if (!use_guided)
    {
      return os_.findRandomPushPose(cur_point_cloud_);
    }
    else
    {
      return os_.getPushVector(cur_color_frame_, cur_depth_frame_,
                               cur_point_cloud_, cur_workspace_mask_);
    }
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

  PoseStamped getTablePlane(XYZPointCloud& cloud)
  {
    XYZPointCloud obj_cloud, table_cloud;
    Eigen::Vector4f table_centroid = pcl_segmenter_.getTablePlane(cloud,
                                                                  obj_cloud,
                                                                  table_cloud);
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
  std::string cam_info_topic_;
  int tracker_count_;
  bool autorun_pcl_segmentation_;
  bool use_guided_pushes_;
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
