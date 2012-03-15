
#include "simple_occupancy_grid/occupancy_grid.h"
#include <stdio.h>


int main (int argc, char *argv[])
{
    ros::init(argc, argv, "occupancy_grid_node");


    ros::NodeHandle nh("~");

    double center_x, center_y, center_z;
    double size_x, size_y, size_z;
    double res_x, res_y, res_z;

    nh.param<double>("center_x", center_x, 0.5);
    nh.param<double>("center_y", center_y, 0.);
    nh.param<double>("center_z", center_z, 1.0);

    nh.param<double>("size_x", size_x, 1.0);
    nh.param<double>("size_y", size_y, 1.0);
    nh.param<double>("size_z", size_z, 0.5);

    nh.param<double>("res_x", res_x, 0.01);
    nh.param<double>("res_y", res_y, 0.01);
    nh.param<double>("res_z", res_z, 0.01);

    occupancy_grid::OccupancyGrid og(center_x, center_y, center_z,
                                     size_x, size_y, size_z,
                                     res_x, res_y, res_z);

    ROS_INFO("Occupancy Grid node is up");

    ros::spin();

}






