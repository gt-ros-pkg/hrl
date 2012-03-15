#pragma once

//STL
#include <string.h>
#include <math.h>

//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl_ros/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/registration/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
//#include <pcl_ros/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>

//TF
#include <tf/transform_listener.h>

//Msgs
#include <book_stacking_msgs/PlaneInfo.h>
#include <book_stacking_msgs/PlaneInfos.h>
#include <book_stacking_msgs/ObjectInfo.h>
#include <book_stacking_msgs/ObjectInfos.h>

typedef pcl::PointXYZ Point;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;


book_stacking_msgs::PlaneInfos getPlanesByNormals(const pcl::PointCloud<Point>& cloud,
					       unsigned int max_planes, 
					       bool cluster_planes,
					       bool use_concave, 
					       bool use_omp=false, 
					       double dist_thresh=0.03, 
					       double max_sac_iterations=1000, 
					       double sac_probability=0.99, 
					       unsigned int min_inliers=1000, 
					       double search_radius=0.1, 
					       double cluster_tolerance=0.1);


void drawPlaneMarkers(const book_stacking_msgs::PlaneInfos& planes, const ros::Publisher& plane_pub, float r=0.0f, float g=1.0f, float b=0.0f);

void drawPlaneMarkers(const std::vector<book_stacking_msgs::PlaneInfo>& planes, const ros::Publisher& plane_pub, float r=0.0f, float g=1.0f, float b=0.0f);

void drawObjectPrisms(const book_stacking_msgs::ObjectInfos& objects, const ros::Publisher& object_pub, const book_stacking_msgs::PlaneInfo& plane, float r=0.0f, float g=0.0f, float b=1.0f);


book_stacking_msgs::ObjectInfos getObjectsOverPlane(const book_stacking_msgs::PlaneInfo& plane, const pcl::PointCloud<Point>& cloud, double prism_min_height=0.0, double prism_max_height=1.0);

geometry_msgs::Point32 calcCentroid(const pcl::PointCloud<Point>& cloud);
