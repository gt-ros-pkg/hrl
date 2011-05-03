#include "pcl_ros/io/bag_io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include <pcl/common/eigen.h>

#include <string>
#include <sstream>
#include <ros/ros.h>
using std::string;
using std::stringstream;

typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

class Segmenter
{
 public:
  void runBag(string filename, string topic_name)
  {
    cloud_cnt_ = 0;
    pcl_ros::BAGReader reader;
    if (! reader.open(filename, topic_name))
    {
      ROS_ERROR_STREAM("Unable to open topic: " << topic_name <<
                       " from bag file: " << filename);
      return;
    }

    sensor_msgs::PointCloud2ConstPtr cloud_msg, cloud_msg_prev;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    do
    {
      cloud_msg_prev = cloud_msg;
      cloud_msg = reader.getNextCloud ();
      if (cloud_msg_prev != cloud_msg)
      {
        pcl::fromROSMsg (*cloud_msg, cloud);

        ROS_INFO_STREAM("Procesing cloud: " << cloud_cnt_);

        processCloud(cloud);

        cloud_cnt_++;
      }
    }
    while (cloud_msg != cloud_msg_prev);
  }

  void processCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    // Output raw cloud for visualization purposes
    stringstream cloud_outname;
    cloud_outname <<  "/home/thermans/data/full_cloud" << cloud_cnt_ << ".pcd";
    pcl::io::savePCDFileASCII(cloud_outname.str(), cloud);
    ROS_INFO_STREAM("Saved " << cloud.points.size()
                    << " data points to pcd file "
                    << cloud_outname.str());

    // Filter Cloud to be just table top height
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered, cloud_z_filtered;
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    z_pass.setInputCloud(
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    z_pass.setFilterFieldName ("z");
    z_pass.setFilterLimits (0.67, 1.3);
    z_pass.filter(cloud_z_filtered);

    // Filter to be just in the  range in front of the robot
    pcl::PassThrough<pcl::PointXYZ> x_pass;
    x_pass.setInputCloud(
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud_z_filtered));
    x_pass.setFilterFieldName ("x");
    x_pass.setFilterLimits (0.25, 1.0);
    x_pass.filter(cloud_filtered);

    stringstream cloud_z_outname;
    cloud_z_outname <<  "/home/thermans/data/z_filtered" << cloud_cnt_ << ".pcd";
    pcl::io::savePCDFileASCII(cloud_z_outname.str(), cloud_z_filtered);
    ROS_INFO_STREAM("Saved " << cloud_z_filtered.points.size()
                    << " data points to pcd file "
                    << cloud_z_outname.str());

    stringstream cloud_x_outname;
    cloud_x_outname <<  "/home/thermans/data/x_filtered" << cloud_cnt_ << ".pcd";
    pcl::io::savePCDFileASCII(cloud_x_outname.str(), cloud_filtered);
    ROS_INFO_STREAM("Saved " <<
                    cloud_filtered.points.size()
                    << " data points to pcd file "
                    << cloud_x_outname.str());

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
    stringstream cloud_plane_outname;
    cloud_plane_outname <<  "/home/thermans/data/plane" << cloud_cnt_ << ".pcd";
    pcl::io::savePCDFileASCII(cloud_plane_outname.str(), plane_cloud);
    ROS_INFO_STREAM("Saved " << plane_cloud.points.size()
                    << " data points to pcd file "
                    << cloud_plane_outname.str());

    // Extract the outliers from the point clouds
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ> objects_cloud;
    pcl::PointIndices plane_outliers;
    extract.setInputCloud (
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud_filtered));
    extract.setIndices (boost::make_shared<pcl::PointIndices> (plane_inliers));
    extract.setNegative (true);
    extract.filter(objects_cloud);
    stringstream cloud_objects_outname;
    cloud_objects_outname <<  "/home/thermans/data/objects" << cloud_cnt_ << ".pcd";
    pcl::io::savePCDFileASCII(cloud_objects_outname.str(), objects_cloud);
    ROS_INFO_STREAM("Saved " << objects_cloud.points.size()
                    << " data points to pcd file "
                    << cloud_objects_outname.str());


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

    // Write point clouds to disk for each cluster
    for (unsigned int i = 0; i < clusters.size(); ++i)
    {
      pcl::PointCloud<pcl::PointXYZ> obj_cloud;
      pcl::copyPointCloud(objects_cloud, clusters[i], obj_cloud);
      stringstream cloud_obj_outname;
      cloud_obj_outname <<  "/home/thermans/data/object" << cloud_cnt_
                        << "-" << i << ".pcd";
      pcl::io::savePCDFileASCII(cloud_obj_outname.str(), obj_cloud);
      ROS_INFO_STREAM("Saved " << obj_cloud.points.size()
                      << " data points to pcd file "
                      << cloud_obj_outname.str());
    }

  }

 protected:
  int cloud_cnt_;
};

int main()
{
  Segmenter seg;
  seg.runBag("/home/thermans/data/table_top2.bag", "/table_cloud");
  return 0;
}
