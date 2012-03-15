#ifndef OCCUPANCY_GRID_OCCUPANCY_GRID_
#define OCCUPANCY_GRID_OCCUPANCY_GRID_


#include <stdint.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Empty.h>

#include "hrl_msgs/FloatArrayBare.h"
#include "hrl_srvs/FloatArray_None.h"


/**
 * @class OccupancyGrid
 * @brief A 3D grid sturcture that stores points as a 3D array.
 * I am going to assume that this class will always be used with ROS.
 * So am writing the visualization code (which uses rviz) within the
 * same class.
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

            unsigned int nX();
            unsigned int nY();
            unsigned int nZ();

            // return flattened array with the occupancy count for
            // each cell of the occupancy grid.
            uint32_t* getOccupancyCountArray();

            /**
             * @brief add list of points (in a ROS FloatArrayBare
             * structure) to the occupancy grid. Not worrying about
             * timestamps for now.  length of  pts_fab is 3N for N
             * points.
             */
            void addPointsUnstamped(const hrl_msgs::FloatArrayBare pts_fab);

            /**
             * @brief add list of points (in a std vector of doubles)
             * to the occupancy grid. Not worrying about timestamps
             * for now.  length of  pts_vec is 3N for N points.
             */
            void addPointsUnstamped(const std::vector<double> pts_vec);

            /**
             * @brief cube markers for cells that have occupancy count >=1
             */
            void publishMarkerArray_simple();


        private:
            unsigned int nx_, ny_, nz_;
            float size_x_, size_y_, size_z_;
            float center_x_, center_y_, center_z_;
            float res_x_, res_y_, res_z_;
            uint32_t *occupancy_count_array_;

            // ROS stuff
            ros::NodeHandle nh_;
            ros::Subscriber sub_cmd_viz_simple_;
            ros::ServiceServer srv_add_points_unstamped_;
            ros::Publisher marker_pub_;

            void publishMarkerArray_simple_cb(const std_msgs::Empty::ConstPtr& msg);

            bool addPointsUnstamped_srv(hrl_srvs::FloatArray_None::Request &req,
                                        hrl_srvs::FloatArray_None::Response &res);
    };
};

#endif


