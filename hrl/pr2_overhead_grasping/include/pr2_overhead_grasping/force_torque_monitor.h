#ifndef PR2_OVERHEAD_GRASPING_FORCE_TORQUE_MONITOR_H
#define PR2_OVERHEAD_GRASPING_FORCE_TORQUE_MONITOR_H
#include <ros/ros.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Bool.h"
#include "pr2_overhead_grasping/SensorPoint.h"
#include "pr2_overhead_grasping/StartFTDetect.h"
#include <std_srvs/Empty.h>
#include <nodelet/nodelet.h>

namespace collision_detection {

  class ForceTorqueMonitor : public nodelet::Nodelet {
    public:
      virtual void onInit();
      void startDetection();
      void stopDetection();

    protected:
      void checkCollision(geometry_msgs::Vector3Stamped::ConstPtr message);
      void saveCollision(pr2_overhead_grasping::SensorPoint::ConstPtr message);
      double z_thresh;
      double collision_time;
      double delay_time;
      int label;
      bool collision_detected;
      bool is_collision;
      int traj_ind;
      ros::Timer delay_timer;

      bool srvStartDetection(pr2_overhead_grasping::StartFTDetect::Request&, pr2_overhead_grasping::StartFTDetect::Response&);
      bool srvStopDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
      void endCollision(const ros::TimerEvent& event);
      ros::Publisher coll_pub;
      ros::Publisher detect_pub;
      ros::Subscriber ft_sub;
      ros::Subscriber sf_sub;
      ros::ServiceServer start_srv;
      ros::ServiceServer stop_srv;
  };
}


#endif // PR2_OVERHEAD_GRASPING_FORCE_TORQUE_MONITOR_H
