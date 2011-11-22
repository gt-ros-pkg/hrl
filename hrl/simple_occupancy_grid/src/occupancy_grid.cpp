

#include "simple_occupancy_grid/occupancy_grid.h"
//#include "point_cloud_ros/OccupancyGrid.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <stdio.h>

namespace occupancy_grid
{
    OccupancyGrid::OccupancyGrid(ros::NodeHandle& nh,
                                 float center_x, float center_y, float center_z,
                                 float size_x, float size_y, float size_z,
                                 float res_x, float res_y, float res_z)
    {
        nx_ = int (size_x / res_x + 0.5);
        ny_ = int (size_y / res_y + 0.5);
        nz_ = int (size_z / res_z + 0.5);

        res_x_ = res_x; 
        res_y_ = res_y; 
        res_z_ = res_z; 

        size_x_ = size_x; 
        size_y_ = size_y; 
        size_z_ = size_z; 

        center_x_ = center_x; 
        center_y_ = center_y; 
        center_z_ = center_z; 

        occupancy_count_array_ = new uint32_t[nx_ * ny_ * nz_];
        for(unsigned int i = 0; i < nx_ * ny_ *nz_; i++)
            occupancy_count_array_[i] = 0;

        marker_pub_ = nh.advertise<visualization_msgs::Marker>("viz", 5);

    }

    OccupancyGrid::~OccupancyGrid()
    {
        delete [] occupancy_count_array_;
    }

    unsigned int OccupancyGrid::nX()
    {
        return nx_;
    }

    unsigned int OccupancyGrid::nY()
    {
        return ny_;
    }

    unsigned int OccupancyGrid::nZ()
    {
        return nz_;
    }

    uint32_t* OccupancyGrid::getOccupancyCountArray()
    {
        return occupancy_count_array_;
    }


    void OccupancyGrid::addPointsUnstamped(const hrl_msgs::FloatArrayBare pt_list)
    {
        float x, y, z;
        int idx_x, idx_y, idx_z;
        float min_x = center_x_ - size_x_ / 2;
        float min_y = center_y_ - size_y_ / 2;
        float min_z = center_z_ - size_z_ / 2;

        for (size_t i = 0; i < pt_list.data.size(); i=i+3)
        {
            x = pt_list.data[i];
            y = pt_list.data[i+1];
            z = pt_list.data[i+2];

            idx_x = int( (x - min_x) / res_x_ + 0.5);
            idx_y = int( (y - min_y) / res_y_ + 0.5);
            idx_z = int( (z - min_z) / res_z_ + 0.5);

            if (idx_x >= 0 and idx_x < (int)nx_ and idx_y >= 0 and \
                    idx_y < (int)ny_ and idx_z >= 0 and idx_z < (int)nz_)
                occupancy_count_array_[idx_x * nz_ * ny_ + idx_y * nz_ + idx_z] += 1;
        }
    }


    void OccupancyGrid::publishMarkerArray_simple()
    {
        visualization_msgs::Marker cube_list_marker;
        cube_list_marker.header.stamp = ros::Time::now();

        // whoever uses this can define a static transform between
        // /occupancy_grid_frame and /torso_lift_link, /world etc.
        cube_list_marker.header.frame_id = "/occupancy_grid_frame";
        cube_list_marker.ns = "occupancy_grid_simple";
        cube_list_marker.id = 0;
        cube_list_marker.action = visualization_msgs::Marker::ADD;
        cube_list_marker.lifetime = ros::Duration();
        cube_list_marker.type = visualization_msgs::Marker::CUBE_LIST;

        cube_list_marker.scale.x = res_x_;
        cube_list_marker.scale.y = res_y_;
        cube_list_marker.scale.z = res_z_;

        std_msgs::ColorRGBA c;
        c.r = 0.;
        c.g = 1.;
        c.b = 1.;
        // for some reason, alpha is common for all the cubes.
        cube_list_marker.color.a = 0.2;

        for(unsigned int x_idx=0; x_idx<nx_; x_idx++)
            for(unsigned int y_idx=0; y_idx<ny_; y_idx++)
                for(unsigned int z_idx=0; z_idx<nz_; z_idx++)
                    if (occupancy_count_array_[x_idx * nz_ * ny_ + y_idx * nz_ + z_idx] > 0)
                    {
                        geometry_msgs::Point pt;
                        pt.x = x_idx*res_x_ + center_x_ - size_x_/2;
                        pt.y = y_idx*res_y_ + center_y_ - size_y_/2;
                        pt.z = z_idx*res_z_ + center_z_ - size_z_/2;
                        cube_list_marker.points.push_back(pt);
                        cube_list_marker.colors.push_back(c);
                    }

        marker_pub_.publish(cube_list_marker);
    }
    
/*
    void OccupancyGrid::fillOccupancyGrid(const sensor_msgs::PointCloud cloud)
    {
        float x, y, z;
        int idx_x, idx_y, idx_z;
        float min_x = center_x_ - size_x_ / 2;
        float min_y = center_y_ - size_y_ / 2;
        float min_z = center_z_ - size_z_ / 2;

        for (size_t i = 0; i < cloud.points.size(); i++)
        {
            x = cloud.points[i].x;
            y = cloud.points[i].y;
            z = cloud.points[i].z;

            idx_x = int( (x - min_x) / res_x_ + 0.5);
            idx_y = int( (y - min_y) / res_y_ + 0.5);
            idx_z = int( (z - min_z) / res_z_ + 0.5);

            if (idx_x >= 0 and idx_x < (int)nx_ and idx_y >= 0 and \
                idx_y < (int)ny_ and idx_z >= 0 and idx_z < (int)nz_)
                data_[idx_x * nz_ * ny_ + idx_y * nz_ + idx_z] += 1;
//                data_[idx_z * nx_ * ny_ + idx_y * nx_ + idx_x] += 1;
        }
    }

    sensor_msgs::PointCloud OccupancyGrid::gridToPoints()
    {
        sensor_msgs::PointCloud cloud;

        for(unsigned int x_idx=0; x_idx<nx_; x_idx++)
            for(unsigned int y_idx=0; y_idx<ny_; y_idx++)
                for(unsigned int z_idx=0; z_idx<nz_; z_idx++)
                    if (data_[x_idx * nz_ * ny_ + y_idx * nz_ + z_idx] > 0)
                    {
                        geometry_msgs::Point32 pt;
                        pt.x = x_idx*res_x_ + center_x_ - size_x_/2;
                        pt.y = y_idx*res_y_ + center_y_ - size_y_/2;
                        pt.z = z_idx*res_z_ + center_z_ - size_z_/2;
                        cloud.points.push_back(pt);
                    }
        return cloud;
    }
*/

};


