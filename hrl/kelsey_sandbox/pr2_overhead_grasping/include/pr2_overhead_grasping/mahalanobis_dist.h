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
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

using namespace std;
using namespace pr2_overhead_grasping;

namespace collision_detection {

  struct DistFinder {
    MatrixXd V;
    VectorXd D;
    VectorXd means;
    void makeInv(MatrixXd& var_mat, VectorXd& c_means, double min_eig_val=0.01) {
      SelfAdjointEigenSolver<MatrixXd> solver(var_mat);
      D = solver.eigenvalues();
      double thresh = D.maxCoeff() * 0.65;
      double max_thresh = D.maxCoeff() * 0.18;
      int rank = D.size();
      cout << "Eigenvalues";
      for(int i=0;i<D.size();i++) 
        printf("%e ", D(i));
      cout << endl;
      for(uint32_t i=0;i<D.size();i++) {
        if((D(i) > thresh || D(i) < max_thresh) && false) {
          //D(i) = 1.0 / min_eig_val;
          D(i) = 0;
          rank--;
        }
        else
          D(i) = 1.0 / D(i);
      }
      ROS_INFO("Rank: %d", rank);
      V = solver.eigenvectors();
      means = c_means;
    }
    double dist(VectorXd& pt) {
      //return pt.array().abs().maxCoeff();
      static VectorXd left_prod;
      left_prod = (pt - means) * V;
      return sqrt((left_prod * D.asDiagonal()).dot(left_prod.transpose()));
    }
  };

  class MahalanobisDist  {
    public:
      MahalanobisDist() {}
      ~MahalanobisDist();
      void loadDataset();
      void loadDataBag(string& data_bag, int label);
      void classifyCallback(const boost::shared_ptr<SensorPoint>& inst);
      static void runTenFold(vector< SensorPoint::ConstPtr >* train_test_data, 
                                         int roc_id,
                                         int num_trees,
                                         vector<map<int, int> >& votes_total,
                                         bool classify_first=true);
      static int findFirstClass(vector<pair<map<int, int>, float > >* votes_list, 
                                int pos_id, float thresh);
      static int findFrequentClass(vector<pair<map<int, int>, float > >* votes_list, 
                                int pos_id, float thresh);
      void onInit();

      void loadCovMat();
      void doMahalanobis();
      double mahalanobisDist(MatrixXd& cov_inv, VectorXd& means, VectorXd& pt);
      void createCovMat();
      void makeInv(MatrixXd& A, MatrixXd& A_inv, double min_eig_val=0.0001);
      void summarizeData();


    protected:

      ros::NodeHandle* nh;
      ros::NodeHandle* nh_priv;

      int num_classes;
      string classifier_name;
      int classifier_id;
      //vector<RandomTree* >* trees;
      void saveCovMat(MatrixXd& var_mat, VectorXd& means);
      DistFinder cov_inv;
      VectorXd means;
      vector< SensorPoint::ConstPtr >* dataset;
      vector<int> labels;
      ros::Subscriber classify_sub;
      ros::Publisher results_pub;
      ros::Publisher loaded_pub;
      boost::thread setup_thread;
      bool classifier_loaded;
  };
}


#endif // PR2_OVERHEAD_GRASPING_RANDOM_FOREST_H
