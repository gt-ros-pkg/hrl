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
             * @param center_{x,y,z} coordinates of the center of the grid
             * @param size_{x,y,z} size of VOI (in meters)
             * @param res_{x,y,z} resolution along the three directions
             */
            OccupancyGrid(float center_x, float center_y, float center_z,
                          float size_x, float size_y, float size_z,
                          float res_x, float res_y, float res_z);

            ~OccupancyGrid();

            inline void markVoxel(unsigned int x, unsigned int y, unsigned int z)
            {
                if(x >= nx_ || y >= ny_ || z >= nz_)
                {
                    ROS_DEBUG("Error, voxel out of bounds.\n");
                    return;
                }
                data_[z * nx_ * ny_ + y * nx_ + x] = MARKED;
            }

            inline void clearVoxel(unsigned int x, unsigned int y, unsigned int z)
            {
                if(x >= nx_ || y >= ny_ || z >= nz_)
                {
                    ROS_DEBUG("Error, voxel out of bounds.\n");
                    return;
                }
                data_[z * nx_ * ny_ + y * nx_ + x] = FREE;
            }

            static VoxelStatus getVoxel(unsigned int x, unsigned int y, unsigned int z,
                    unsigned int nx, unsigned int ny, unsigned int nz, const uint32_t* data)
            {
                if(x >= nx || y >= ny || z >= nz)
                {
                    ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
                    return UNKNOWN;
                }
                unsigned int voxel_status = data[z * nx * ny + y * nx + x];
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

            unsigned int nx_, ny_, nz_;
            float size_x_, size_y_, size_z_;
            float center_x_, center_y_, center_z_;
            float res_x_, res_y_, res_z_;
            uint32_t *data_;
    };
};

#endif


