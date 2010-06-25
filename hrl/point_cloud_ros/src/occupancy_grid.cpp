#include <point_cloud_ros/occupancy_grid.h>
#include <sys/time.h>
#include <ros/console.h>

namespace occupancy_grid
{
    OccupancyGrid::OccupancyGrid(float center_x, float center_y, float center_z,
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

        data_ = new uint32_t[nx_ * ny_ * nz_];
        for(unsigned int i = 0; i < nx_ * ny_ *nz_; i++)
            data_[i] = 0;
    }

    OccupancyGrid::~OccupancyGrid()
    {
        delete [] data_;
    }

    VoxelStatus OccupancyGrid::getVoxel(unsigned int x, unsigned int y, unsigned int z)
    {
        if(x >= nx_ || y >= ny_ || z >= nz_)
        {
            ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
            return UNKNOWN;
        }

        unsigned int voxel_status = data_[z * nx_ * ny_ + y * nx_ + x];
        if (voxel_status == FREE)
            return FREE;
        if (voxel_status == MARKED)
            return MARKED;
        return UNKNOWN;
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

    uint32_t* OccupancyGrid::getData()
    {
        return data_;
    }
    
    void OccupancyGrid::fillOccupancyGrid(pcl::PointCloud<pcl::PointXYZ> cloud)
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
                data_[idx_z * nx_ * ny_ + idx_y * nx_ + idx_x] += 1;
        }
    }

    void OccupancyGrid::printOccupancyGrid()
    {
        for(unsigned int z = 0; z < nz_; z++)
        {
            printf("Layer z = %d:\n",z);
            for(unsigned int y = 0; y < ny_; y++)
            {
                for(unsigned int x = 0 ; x < nx_; x++)
                    printf((getVoxel(x, y, z)) == occupancy_grid::MARKED? "#" : ".");
                printf("|\n");
            } 
        }
    }
};

int main(int argc, char *argv[])
{
    ROS_INFO("Initializing voxel grid.\n");
    occupancy_grid::OccupancyGrid *v = new occupancy_grid::OccupancyGrid(0, 0, 0, 0.3, 0.4, 0.2, 0.05, 0.05, 0.05);

    //Visualize the output
    v->printOccupancyGrid();
}


