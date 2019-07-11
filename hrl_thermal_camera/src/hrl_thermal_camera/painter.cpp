#include <ros/ros.h>
#include "pcloud_painter.h"

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "pcloud_painter");
  ros::NodeHandle nh("~");
  tf::TransformListener tf_listener;

  PCloud_Painter pc_p(nh, tf_listener);

  pc_p.spin();

  return 0;
}



