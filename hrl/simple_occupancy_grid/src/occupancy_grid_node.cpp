
#include "simple_occupancy_grid/occupancy_grid.h"
#include <stdio.h>


int main (int argc, char *argv[])
{
    ros::init(argc, argv, "occupancy_grid_node");

    occupancy_grid::OccupancyGrid og(0.5, 0., 1.0,
                                     0.5, 0.5, 0.5,
                                     0.02, 0.02, 0.02);

    hrl_msgs::FloatArrayBare fa;

    fa.data.push_back(0.5);
    fa.data.push_back(0.);
    fa.data.push_back(1.);

    fa.data.push_back(0.6);
    fa.data.push_back(0.);
    fa.data.push_back(1.);

    fa.data.push_back(0.4);
    fa.data.push_back(0.);
    fa.data.push_back(1.);

    fa.data.push_back(0.5);
    fa.data.push_back(0.1);
    fa.data.push_back(1.);

    fa.data.push_back(0.5);
    fa.data.push_back(-0.1);
    fa.data.push_back(1.);

    fa.data.push_back(0.5);
    fa.data.push_back(0.);
    fa.data.push_back(1.1);

    fa.data.push_back(0.5);
    fa.data.push_back(0.);
    fa.data.push_back(0.9);

    og.addPointsUnstamped(fa);

    ROS_INFO("Occupancy Grid node is up");

    ros::spin();

}






