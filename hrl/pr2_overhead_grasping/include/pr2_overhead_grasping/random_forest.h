#ifndef PR2_OVERHEAD_GRASPING_RANDOM_FOREST_H
#define PR2_OVERHEAD_GRASPING_RANDOM_FOREST_H
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
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
#include "pr2_overhead_grasping/FoldData.h"
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

  typedef VectorXf Dynamic1D;

  const int NUM_ATTRS = 1120;

  class RandomTree { 
    public:
      int d_tree_num;
      int num_classes;
      vector< SensorPoint::Ptr >* dataset;

      RandomTree(int c_d_tree_num);
      RandomTree(RandomTreeMsg::Ptr);
      void growTree(vector< SensorPoint::Ptr >* c_dataset,
                            vector<int>* inds);
      int classifyInstance(SensorPoint::Ptr inst);
      void writeTree(string& bag_file, bool is_first);

      bool is_abs;

    protected:
      
      RandomTreeMsg::Ptr rand_tree;

      bool attrCompare(int inst_i, int inst_j, int attr);
      void findBestSplit(vector<int>* insts, vector<int>& attrs, pair<int, float>& ret);
      void splitNode(vector<int>* node_inds, 
                     pair<int, float>& split_pt,
                     pair<vector<int>*, vector<int>* >& split_nodes);
  };

  class RandomForest  {
    public:
      RandomForest() {}
      ~RandomForest();
      void loadDataset();
      void loadDataBag(string& data_bag, int label);
      void growForest(vector< SensorPoint::Ptr >* c_dataset,
                            vector<int>* inds, int c_num_trees=100);
      void growWriteForest();
      void loadForest();
      void collectVotes(SensorPoint::Ptr inst, map<int, int>& class_votes);
      void writeForest();
      void writeForest(string file);
      void classifyCallback(const boost::shared_ptr<SensorPoint>& inst);
      //void runTests(vector< SensorPoint::ConstPtr >* test_data, vector<int>* test_labels, int num_roc);
      void setDataset(vector< SensorPoint::Ptr >* datas);
      static void runTenFold(vector< SensorPoint::Ptr >* train_test_data, 
                                         int roc_id,
                                         int num_trees,
                                         vector<map<int, int> >& votes_total,
                                         bool classify_first=true);
      static int findFirstClass(vector<pair<map<int, int>, float > >* votes_list, 
                                int pos_id, float thresh);
      static int findFrequentClass(vector<pair<map<int, int>, float > >* votes_list, 
                                int pos_id, float thresh);
      void variableImportance();
      void randomPermuteData();
      void onInit();

      void loadCovMat();
      void doMahalanobis();
      double mahalanobisDist(LDLT<MatrixXd>* cov_inv, VectorXd& means, VectorXd& pt);
      void createCovMat();


    protected:

      ros::NodeHandle* nh;
      ros::NodeHandle* nh_priv;

      int num_trees, num_classes;
      string classifier_name;
      int classifier_id;
      //vector<RandomTree* >* trees;
      RandomTree** trees;
      vector<vector<uint32_t> > oobs;
      LDLT<MatrixXd>* cov_inv;
      VectorXd means;
      vector< SensorPoint::Ptr >* dataset;
      vector<int> labels;
      ros::Subscriber classify_sub;
      ros::Publisher results_pub;
      ros::Publisher loaded_pub;
      boost::thread setup_thread;
      bool trees_loaded;
      bool is_abs;
  };
}


#endif // PR2_OVERHEAD_GRASPING_RANDOM_FOREST_H
