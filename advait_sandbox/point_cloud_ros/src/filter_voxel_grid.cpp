
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/filters/voxel_grid.h"

int main (int argc, char** argv)
{
    sensor_msgs::PointCloud2 cloud, cloud_filtered;

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read ("table_scene_lms400.pcd", cloud);

    ROS_INFO ("PointCloud before filtering: %d data points (%s).", cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());

    // Create the filtering object
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sor.setInputCloud (boost::make_shared<sensor_msgs::PointCloud2> (cloud));
    sor.setLeafSize (0.01, 0.01, 0.01);
    sor.filter (cloud_filtered);

    ROS_INFO ("PointCloud after filtering: %d data points (%s).", cloud_filtered.width * cloud_filtered.height, pcl::getFieldsList (cloud_filtered).c_str ());

    pcl::PCDWriter writer;
    writer.write ("table_scene_lms400_downsampled.pcd", cloud_filtered, false);

    return (0);
}


