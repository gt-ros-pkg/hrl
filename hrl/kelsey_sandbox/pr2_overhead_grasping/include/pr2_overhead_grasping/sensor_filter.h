#ifndef PR2_OVERHEAD_GRASPING_SENSOR_FILTER_H
#define PR2_OVERHEAD_GRASPING_SENSOR_FILTER_H
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "pr2_msgs/AccelerometerState.h"
#include "pr2_msgs/PressureState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "boost/thread/thread.hpp"
//#include "sensor_msgs/Float32MultiArray.h"
//#include "sensor_msgs/MultiArrayDimension.h"
#include "pr2_overhead_grasping/SensorPoint.h"
#include "roslib/Header.h"
#include <ros/package.h>
#include <math.h>
#include <nodelet/nodelet.h>

using namespace pr2_msgs;
using namespace pr2_controllers_msgs;
using namespace sensor_msgs;

namespace collision_detection {
  float compGauss(float sigma, float x);

  class SensorFilter : public nodelet::Nodelet {
    public:
      virtual void onInit();
      float* sigmas;
      void startOnline();
      ~SensorFilter();

    protected:
      char arm_;
      int rate;
      int num_sensors;
      int num_scales;
      long buffer_len;

      long cur_iter;
      float** buffer; // holds raw data
      float** filters; // holds gauss filters
      float*** filtered_buffer; // holds raw data run through gauss filters
                                // time x sensor x scale
      //Float32MultiArray sensor_data;

      //void stopOnline();
      //float*** runOffline(float** sensor_streams);
      void recordData();
      void zeroBuffer();
      void applyFilter();
      
      ros::Subscriber js_sub;
      ros::Subscriber acc_sub;
      ros::Subscriber press_sub;
      ros::Subscriber error_sub;

      // most recent messages
      JointState::ConstPtr js_msg;
      AccelerometerState::ConstPtr acc_msg;
      PressureState::ConstPtr press_msg;
      JointTrajectoryControllerState::ConstPtr error_msg;

      void jsCallback(JointState::ConstPtr message);
      void accCallback(AccelerometerState::ConstPtr message);
      void pressCallback(PressureState::ConstPtr message);
      void errorCallback(JointTrajectoryControllerState::ConstPtr message);

      bool js_in, acc_in, press_in, error_in;

      boost::thread sf_thread;
  };
}

#endif // PR2_OVERHEAD_GRASPING_SENSOR_FILTER_H
