#include "pcl_ros/io/bag_io.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"

int main (int argc, char** argv)
{
  sensor_msgs::PointCloud2ConstPtr cloud_blob, cloud_blob_prev;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for(int i=0; i < argc; i++)
  {
      printf("%s", argv[i]);
  }

  pcl_ros::BAGReader reader;
  if (!reader.open ("./table_top2.bag", "/table_cloud"))
  {
    ROS_ERROR ("Couldn't read file test_io_bag.bag. Try doing a 'make tests' in pcl, or download the file manually from http://pr.willowgarage.com/data/pcl/test_io_bag.bag");
    return (-1);
  }

  int cnt = 0;
  do
  {
    cloud_blob_prev = cloud_blob;
    cloud_blob = reader.getNextCloud ();
    if (cloud_blob_prev != cloud_blob)
    {
      pcl::fromROSMsg (*cloud_blob, cloud);
      ROS_INFO ("PointCloud with %d data points and frame %s (%f) received.", (int)cloud.points.size (), cloud.header.frame_id.c_str (), cloud.header.stamp.toSec ()); 
      pcl::io::savePCDFileASCII ("table_cloud.pcd", cloud);
      //if (cnt == 0)
      //{
      //    for(int i=0; i < cloud.points.size();i++)
      //    {
      //    }
      //}
      cnt++;
    }
  }
  while (cloud_blob != cloud_blob_prev);

  ROS_INFO ("Total number of PointCloud messages processed: %d", cnt);

  return (0);
}

