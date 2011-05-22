#ifndef PR2_OVERHEAD_GRASPING_COLLISION_MONITOR_H
#define PR2_OVERHEAD_GRASPING_COLLISION_MONITOR_H
#include <numeric>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "pr2_msgs/AccelerometerState.h"
#include "pr2_msgs/PressureState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "boost/thread/thread.hpp"
//#include "sensor_msgs/Float32MultiArray.h"
//#include "sensor_msgs/MultiArrayDimension.h"
#include "pr2_overhead_grasping/SensorPoint.h"
#include "pr2_overhead_grasping/CollisionDescription.h"
#include "pr2_overhead_grasping/ClassVotes.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "roslib/Header.h"
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <nodelet/nodelet.h>

using namespace std;

namespace collision_detection {

  class CollisionMonitor : public nodelet::Nodelet {
    public:
      virtual void onInit();
      void startDetection();
      void triggerCollision();
      ~CollisionMonitor(); 

    protected:
      vector<int> detect_buffer;
      vector<int> sigma_buffer;
      int ind, sigma_ind;

      bool rf_on, sigma_on, error_on;
      int rf_thresh;
      int rf_buf_len;
      double start_time;
      double sigma_thresh;
      int sigma_buf_len;
      int sigma_num;
      vector<double> min_errors, max_errors;

      bool collision_detected;
      bool is_training;
      int collision_id;
      int collision_thresh;

      void checkCollision(pr2_overhead_grasping::ClassVotes::ConstPtr message);
      void describeCollision();
      void sigmaCallback(std_msgs::Float64::ConstPtr msg);
      void errorCallback(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr message);
      vector<pr2_overhead_grasping::ClassVotes::ConstPtr> msg_buffers;
      bool srvStartDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
      bool srvStopDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
      bool srvTriggerCollision(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
      ros::Publisher detect_pub;
      ros::Publisher descript_pub;
      ros::Subscriber sf_sub;
      ros::Subscriber sigma_sub;
      ros::Subscriber error_sub;
      vector<ros::Subscriber> class_subs;
      ros::ServiceServer start_srv;
      ros::ServiceServer stop_srv;
      ros::ServiceServer trig_srv;
      ros::ServiceClient zero_sigma;

      boost::thread descript_thread;
  };
}


#endif // PR2_OVERHEAD_GRASPING_COLLISION_MONITOR_H
