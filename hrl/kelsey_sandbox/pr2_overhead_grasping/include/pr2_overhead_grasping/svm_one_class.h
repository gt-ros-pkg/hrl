#ifndef PR2_OVERHEAD_GRASPING_RANDOM_FOREST_H
#define PR2_OVERHEAD_GRASPING_RANDOM_FOREST_H
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include "pr2_msgs/AccelerometerState.h"
#include "pr2_msgs/PressureState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "boost/thread/thread.hpp"
#include "boost/format.hpp"
#include "boost/foreach.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
//#include "sensor_msgs/Float32MultiArray.h"
//#include "sensor_msgs/MultiArrayDimension.h"
#include "pr2_overhead_grasping/SensorPoint.h"
#include "pr2_overhead_grasping/ClassVotes.h"
#include "pr2_overhead_grasping/CollisionDescription.h"
#include "pr2_overhead_grasping/RandomTreeMsg.h"
#include "pr2_overhead_grasping/CovarianceMatrix.h"
#include "roslib/Header.h"
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <nodelet/nodelet.h>
#include <algorithm>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Cholesky>
#include "svm.h"
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

using namespace std;
using namespace pr2_overhead_grasping;

namespace collision_detection {

  class SVMOneClass  {
    public:
      SVMOneClass() {}
      ~SVMOneClass();
      void loadDataset();
      void loadDataBag(string& data_bag, int label);
      void onInit();

      void generateModel();

    protected:

      ros::NodeHandle* nh;
      ros::NodeHandle* nh_priv;

      int num_classes;
      string classifier_name;
      int classifier_id;
      vector< SensorPoint::ConstPtr >* dataset;
      ros::Subscriber classify_sub;
      ros::Publisher results_pub;
      ros::Publisher loaded_pub;
      boost::thread setup_thread;
      bool classifier_loaded;
  };
}


#endif // PR2_OVERHEAD_GRASPING_RANDOM_FOREST_H
