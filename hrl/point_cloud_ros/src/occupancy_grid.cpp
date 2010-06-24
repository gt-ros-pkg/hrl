#include <point_cloud_ros/occupancy_grid.h>
#include <sys/time.h>
#include <ros/console.h>

namespace occupancy_grid
{
    OccupancyGrid::OccupancyGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z)
    {
        size_x_ = size_x; 
        size_y_ = size_y; 
        size_z_ = size_z; 

        data_ = new uint32_t[size_x_ * size_y_ * size_z_];
        for(unsigned int i = 0; i < size_x_ * size_y_ *size_z_; i++)
            data_[i] = 0;
    }

    OccupancyGrid::~OccupancyGrid()
    {
        delete [] data_;
    }

    VoxelStatus OccupancyGrid::getVoxel(unsigned int x, unsigned int y, unsigned int z)
    {
        if(x >= size_x_ || y >= size_y_ || z >= size_z_)
        {
            ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
            return UNKNOWN;
        }

        unsigned int voxel_status = data_[z * size_x_ * size_y_ + y * size_x_ + x];
        if (voxel_status == FREE)
            return FREE;
        if (voxel_status == MARKED)
            return MARKED;
        return UNKNOWN;
    }


    unsigned int OccupancyGrid::sizeX()
    {
        return size_x_;
    }

    unsigned int OccupancyGrid::sizeY()
    {
        return size_y_;
    }

    unsigned int OccupancyGrid::sizeZ()
    {
        return size_z_;
    }

    void OccupancyGrid::printOccupancyGrid()
    {
        for(unsigned int z = 0; z < size_z_; z++)
        {
            printf("Layer z = %d:\n",z);
            for(unsigned int y = 0; y < size_y_; y++)
            {
                for(unsigned int x = 0 ; x < size_x_; x++)
                    printf((getVoxel(x, y, z)) == occupancy_grid::MARKED? "#" : ".");
                printf("|\n");
            } 
        }
    }
};

int main(int argc, char *argv[])
{
    ROS_INFO("Initializing voxel grid.\n");
    int size_x = 3, size_y = 4, size_z = 5;
    occupancy_grid::OccupancyGrid *v = new occupancy_grid::OccupancyGrid(size_x, size_y, size_z);

    //Visualize the output
    v->printOccupancyGrid();
}


