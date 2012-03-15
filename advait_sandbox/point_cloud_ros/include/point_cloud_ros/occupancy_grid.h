#ifndef OCCUPANCY_GRID_OCCUPANCY_GRID_
#define OCCUPANCY_GRID_OCCUPANCY_GRID_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>

/**
 * @class OccupancyGrid
 * @brief A 3D grid sturcture that stores points as a 3D array.
 */
namespace occupancy_grid
{
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

            void fillOccupancyGrid(const sensor_msgs::PointCloud cloud);

            sensor_msgs::PointCloud gridToPoints();

            unsigned int nX();
            unsigned int nY();
            unsigned int nZ();

            uint32_t* getData();

        private:
            unsigned int nx_, ny_, nz_;
            float size_x_, size_y_, size_z_;
            float center_x_, center_y_, center_z_;
            float res_x_, res_y_, res_z_;
            uint32_t *data_;
    };
};

#endif


