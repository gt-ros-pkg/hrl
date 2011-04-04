/*
   Copied from pcl tutorial.

*/

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

int main (int argc, char** argv)
{
  sensor_msgs::PointCloud2 cloud_blob;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if (pcl::io::loadPCDFile ("test_pcd.pcd", cloud_blob) == -1)
  {
    ROS_ERROR ("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
  point_cloud::fromMsg (cloud_blob, cloud);

  std::cout << "Size of point cloud:" << cloud.points.size() << std::endl;
//  for (size_t i = 0; i < cloud.points.size (); ++i)
//    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}

