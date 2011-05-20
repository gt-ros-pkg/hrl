// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_assembler/AssembleScans.h>
#include <tabletop_pushing/PushPose.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/io/bag_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include <pcl/features/feature.h>
#include <pcl/common/eigen.h>

#include <string>
#include <sstream>
#include <ros/ros.h>
#include <exception>

using std::string;
using std::stringstream;
using tabletop_pushing::PushPose;
typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

class Segmenter
{
 public:
  Segmenter(ros::NodeHandle n) : n_(n), tf_()
  {
    // Create the service client for calling the assembler
    assembler_client_ = n_.serviceClient<laser_assembler::AssembleScans>(
        "tabletop_pushing_asemble_scans");
    push_point_server_ = n_.advertiseService("get_push_pose",
                                             &Segmenter::getPushPose,
                                             this);
    ros::NodeHandle n_private("~");
    std::string point_cloud_topic_def = "/narrow_stereo_textured/points";
    n_private.param("point_cloud_topic", point_cloud_topic_,
                    point_cloud_topic_def);
    n_private.param("use_point_cloud_2", use_point_cloud_2_, false);
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

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    // TODO: Make work with laser too
    sensor_msgs::PointCloud cloud_msg;
    laser_assembler::AssembleScans srv;
    sensor_msgs::PointCloud2 cloud2_msg;
    try
    {
      if (req.use_laser)
      {
        srv.request.end = ros::Time::now();
        srv.request.begin = srv.request.end - ros::Duration(2.0);

        ROS_INFO_STREAM("Attempting to get point cloud");
        if (assembler_client_.call(srv))
        {
          cloud_msg = srv.response.cloud;
        }
        else
        {
          ROS_ERROR("Error making service call to assembler\n") ;
          return false;
        }
      }
      else if (use_point_cloud_2_)
      {
        // Use the stereo instead of the laser
        ROS_INFO_STREAM("Waiting for PointCloud2 message");
        cloud2_msg = *ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            point_cloud_topic_, n_, ros::Duration(5.0));
        sensor_msgs::convertPointCloud2ToPointCloud(cloud2_msg, cloud_msg);
        tf_.transformPointCloud("/torso_lift_link", cloud_msg, cloud_msg);
        ROS_INFO_STREAM("Got PointCloud2 message");
      }
      else
      {
        // Use the stereo instead of the laser
        ROS_INFO_STREAM("Waiting for PointCloud message");
        cloud_msg = *ros::topic::waitForMessage<sensor_msgs::PointCloud>(
            point_cloud_topic_, n_, ros::Duration(3.0));
        tf_.transformPointCloud("/torso_lift_link", cloud_msg, cloud_msg);
        ROS_INFO_STREAM("Got PointCloud message");
      }
      sensor_msgs::convertPointCloudToPointCloud2(cloud_msg, cloud2_msg);

      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(cloud2_msg, cloud);
      ROS_INFO_STREAM("Segmenting point cloud");
      res.push_pose = processCloud(cloud);

      if (res.push_pose.pose.position.x == 0.0 &&
          res.push_pose.pose.position.y == 0.0 &&
          res.push_pose.pose.position.z == 0.0)
      {
        res.invalid_push_pose = true;
      }
      else
      {
        res.invalid_push_pose = false;
      }
      return true;
    } catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Exception in segmentation: " << e.what());
      return false;
    }

  }

  geometry_msgs::PoseStamped processCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered, cloud_z_filtered,
        cloud_downsampled;

    // Downsample using a voxel grid for faster performance
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    downsample.setLeafSize(0.005, 0.005, 0.005);
    downsample.filter(cloud_downsampled);
    ROS_INFO_STREAM("Voxel Downsampled Cloud");

    // Filter Cloud to be just table top height
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setInputCloud(
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_downsampled));
    z_pass.setFilterFieldName ("z");
    z_pass.setFilterLimits (-1.0, 1.0);
    z_pass.filter(cloud_z_filtered);
    ROS_INFO_STREAM("Filtered z");

    // Filter to be just in the  range in front of the robot
    pcl::PassThrough<pcl::PointXYZ> x_pass;
    x_pass.setInputCloud(
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_z_filtered));
    x_pass.setFilterFieldName ("x");
    x_pass.setFilterLimits (0.5, 1.5);
    x_pass.filter(cloud_filtered);
    ROS_INFO_STREAM("Filtered x");

    // Segment the tabletop from the points using RANSAC plane fitting
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices plane_inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType (pcl::SACMODEL_PLANE);
    plane_seg.setMethodType (pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold (0.01);
    plane_seg.setInputCloud (
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_filtered));
    plane_seg.segment(plane_inliers, coefficients);

    // Extract the plane members into their own point cloud and save them
    // into a PCD file
    pcl::PointCloud<pcl::PointXYZ> plane_cloud;
    pcl::copyPointCloud(cloud_filtered, plane_inliers, plane_cloud);

    // Extract the outliers from the point clouds
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ> objects_cloud;
    pcl::PointIndices plane_outliers;
    extract.setInputCloud (
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud_filtered));
    extract.setIndices (boost::make_shared<pcl::PointIndices> (plane_inliers));
    extract.setNegative (true);
    extract.filter(objects_cloud);

    // TODO: This would be better if we first filter out points below the plane,
    // so that only objects on top of the table are clustered

    // Cluster the objects based on euclidean distance
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_cluster;
    KdTreePtr clusters_tree =
        boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
    pcl_cluster.setClusterTolerance(0.025);
    pcl_cluster.setMinClusterSize(100);
    pcl_cluster.setSearchMethod(clusters_tree);
    pcl_cluster.setInputCloud(
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(objects_cloud));
    pcl_cluster.extract (clusters);
    ROS_INFO_STREAM("Number of clusters found matching the given constraints: "
                    << clusters.size());

    // Examine the clusters and find the nearest centroid
    float min_x = 10000;
    Eigen::Vector4f close;
    for (unsigned int i = 0; i < clusters.size(); ++i)
    {
      pcl::PointCloud<pcl::PointXYZ> obj_cloud;
      pcl::copyPointCloud(objects_cloud, clusters[i], obj_cloud);

      Eigen::Vector4f xyz_centroid;
      pcl::compute3DCentroid(obj_cloud, xyz_centroid);

      if (xyz_centroid[0]+abs(xyz_centroid[1]) < min_x)
      {
        close = xyz_centroid;
        min_x = close[0]+abs(close[1]);
      }
    }
    geometry_msgs::PoseStamped p;
    if (clusters.size() < 1)
    {
      ROS_ERROR_STREAM("No object clusters found! Returning empty push_pose");
      return p;
    }

    ROS_INFO_STREAM("Nearest centroid is at: (" << close[0] << ", " << close[1]
                    << ", " << close[2] << ")");
    p.pose.position.x = close[0];
    p.pose.position.y = close[1];
    p.pose.position.z = close[2];

    // TODO: Find orientation from the centroid
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 0;

    p.header.frame_id = "torso_lift_link";
    return p;
  }

 protected:
  ros::NodeHandle n_;
  tf::TransformListener tf_;
  ros::Subscriber pc_sub_;
  ros::ServiceClient assembler_client_;
  ros::ServiceServer push_point_server_;
  std::string point_cloud_topic_;
  bool use_point_cloud_2_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tabletop_laser_segmenter_node");
  ros::NodeHandle n;
  Segmenter seg(n);
  seg.spin();
  return 0;
}
