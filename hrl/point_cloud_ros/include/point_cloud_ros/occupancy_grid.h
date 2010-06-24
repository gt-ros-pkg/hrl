#ifndef OCCUPANCY_GRID_OCCUPANCY_GRID_
#define OCCUPANCY_GRID_OCCUPANCY_GRID_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <algorithm>
#include <ros/console.h>
#include <ros/assert.h>


/**
 * @class OccupancyGrid
 * @brief A 3D grid sturcture that stores points as a 3D array.
 */
namespace occupancy_grid
{
    enum VoxelStatus
    {
        FREE = 0,
        UNKNOWN = 1,
        MARKED = 2,
    };

    class OccupancyGrid
    {
        public:
            /**
             * @brief  Constructor for a voxel grid
             * @param size_x The x size of the grid
             * @param size_y The y size of the grid
             * @param size_z The z size of the grid
             */
            OccupancyGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z);

            ~OccupancyGrid();

            inline void markVoxel(unsigned int x, unsigned int y, unsigned int z)
            {
                if(x >= size_x_ || y >= size_y_ || z >= size_z_)
                {
                    ROS_DEBUG("Error, voxel out of bounds.\n");
                    return;
                }
                data_[z * size_x_ * size_y_ + y * size_x_ + x] = MARKED;
            }

            inline void clearVoxel(unsigned int x, unsigned int y, unsigned int z)
            {
                if(x >= size_x_ || y >= size_y_ || z >= size_z_)
                {
                    ROS_DEBUG("Error, voxel out of bounds.\n");
                    return;
                }
                data_[z * size_x_ * size_y_ + y * size_x_ + x] = FREE;
            }

            static VoxelStatus getVoxel(unsigned int x, unsigned int y, unsigned int z,
                    unsigned int size_x, unsigned int size_y, unsigned int size_z, const uint32_t* data)
            {
                if(x >= size_x || y >= size_y || z >= size_z)
                {
                    ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
                    return UNKNOWN;
                }
                unsigned int voxel_status = data[z * size_x * size_y + y * size_x + x];
                if (voxel_status == FREE)
                    return FREE;
                if (voxel_status == MARKED)
                    return MARKED;
                return UNKNOWN;
            }

            VoxelStatus getVoxel(unsigned int x, unsigned int y, unsigned int z);

            void printOccupancyGrid();
            unsigned int sizeX();
            unsigned int sizeY();
            unsigned int sizeZ();

        private:

            inline int sign(int i)
            {
                return i > 0 ? 1 : -1;
            }

            inline unsigned int max(unsigned int x, unsigned int y)
            {
                return x > y ? x : y;
            }

            unsigned int size_x_, size_y_, size_z_;
            uint32_t *data_;
    };
};

#endif


