
#include "point_cloud_ros/occupancy_grid.h"
#include "point_cloud_ros/OccupancyGrid.h"
#include <ros/console.h>
#include "pcl/io/pcd_io.h"
#include "sensor_msgs/point_cloud_conversion.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "occupancy_grid_node");
    ros::NodeHandle n;

    ros::Publisher og_pub = n.advertise<point_cloud_ros::OccupancyGrid>("occupancy_grid", 1);
    ros::Publisher pc_pub = n.advertise<sensor_msgs::PointCloud2>("test_point_cloud", 1);

    sensor_msgs::PointCloud2 cloud_blob;
    sensor_msgs::PointCloud cloud;

    if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
    {
        ROS_ERROR ("Couldn't read file %s", argv[1]);
        return (-1);
    }
    ROS_INFO ("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), pcl::getFieldsList (cloud_blob).c_str ());

    cloud_blob.header.frame_id = "base_link";
    cloud_blob.header.stamp = ros::Time::now();
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_blob, cloud);

    ROS_INFO("Computing the centroid of the point cloud");
    float cx = 0;
    float cy = 0;
    float cz = 0;
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
        cx += cloud.points[i].x;
        cy += cloud.points[i].y;
        cz += cloud.points[i].z;
    }

    cx = cx / cloud.points.size();
    cy = cy / cloud.points.size();
    cz = cz / cloud.points.size();
    ROS_INFO("Centroid of the point cloud: (%.2f, %.2f, %.2f)", cx, cy, cz);

    float rx, ry, rz;
    rx = 0.02;
    ry = 0.02;
    rz = 0.02;

    float sx, sy, sz;
    sx = 1.5;
    sy = 1.5;
    sz = 1.5;
    occupancy_grid::OccupancyGrid *v = new
        occupancy_grid::OccupancyGrid(cx, cy, cz, sx, sy, sz, rx, ry, rz);

    ROS_INFO("Before filling OccupancyGrid");
    v->fillOccupancyGrid(cloud);
    ROS_INFO("After filling OccupancyGrid");

    point_cloud_ros::OccupancyGrid og_msg;

    uint32_t* d = v->getData();
    int nCells = v->nX() * v->nY() * v->nZ();
    og_msg.data.resize(nCells);
    for (int i=0; i<nCells; i++)
        og_msg.data[i] = d[i];



    og_msg.header.frame_id = "base_link";
    og_msg.header.stamp = ros::Time::now();
    og_msg.center.x = cx;
    og_msg.center.y = cy;
    og_msg.center.z = cz;

    og_msg.resolution.x = rx;
    og_msg.resolution.y = ry;
    og_msg.resolution.z = rz;

    og_msg.grid_size.x = sx;
    og_msg.grid_size.y = sy;
    og_msg.grid_size.z = sz;

    og_msg.occupancy_threshold = 1;

    og_pub.publish(og_msg);

    for (int i=0; i<5; i++)
    {
        cloud.header.stamp = ros::Time::now();
        cloud_blob.header.stamp = ros::Time::now();
        ROS_INFO("Iteration number: %d\n", i);
        pc_pub.publish(cloud_blob);
        ros::Duration(5).sleep(); // sleep for half a second
    }

}


