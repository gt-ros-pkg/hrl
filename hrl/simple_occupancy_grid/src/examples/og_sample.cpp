
#include "simple_occupancy_grid/occupancy_grid.h"

#include "hrl_msgs/FloatArrayBare.h"

#include <ros/ros.h>
#include <ros/console.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "occupancy_grid_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Hello World");

    occupancy_grid::OccupancyGrid og(nh,
                                     0.5, 0., 1.0,
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


    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        og.publishMarkerArray_simple();
        loop_rate.sleep();
    }

}


