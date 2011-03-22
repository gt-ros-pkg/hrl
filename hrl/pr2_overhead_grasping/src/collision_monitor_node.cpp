#include "pr2_overhead_grasping/sensor_filter.h"

using namespace pr2_overhead_grasping;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_monitor_node");
  SensorFilter sf('l');

  sf.startOnline();
  

  return 0;
}

