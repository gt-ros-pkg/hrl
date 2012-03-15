
#include <string>

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_ann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

  // std::string topic = nh_.resolveName("cloud_new_in");
  // ROS_INFO("Tabletop detection service called; waiting for a point_cloud2 on topic %s", topic.c_str());

  sensor_msgs::PointCloud2::ConstPtr recent_cloud = 
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>('pr2_segment_object', nh_, ros::Duration(6.0));
              
  if (!recent_cloud)
  {
    ROS_ERROR("Cloud Transformer: no point_cloud2 has been received");
  }

  ROS_INFO("Point cloud received; processing");
  if (!processing_frame_.empty())
  {
    //convert cloud to base link frame
    sensor_msgs::PointCloud old_cloud;  
    sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
    try
    {
      listener_.transformPointCloud('torso_lift_link', old_cloud, old_cloud);    
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Failed to transform cloud from frame %s into frame %s", old_cloud.header.frame_id.c_str(), 
		'torso_lift_link');
    }

    sensor_msgs::PointCloud2 converted_cloud;
    sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);
    ROS_INFO("Input cloud converted to %s frame", processing_frame_.c_str());
    //republish cloud here now for python processing stuff
    processCloud(converted_cloud, request.return_clusters, request.return_models, response.detection);
  }

  return 1;
}
