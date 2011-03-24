#include "pr2_overhead_grasping/svm_one_class.h"
//#include <pluginlib/class_list_macros.h>
#include <omp.h>
#include <stdio.h>
#include <signal.h>
//PLUGINLIB_DECLARE_CLASS(collision_detection, mahalanobis_dist, collision_detection::SVMOneClass, nodelet::Nodelet)

using namespace std;
using namespace pr2_overhead_grasping;
using namespace std_msgs;


namespace collision_detection {

  void SVMOneClass::loadDataset() {
    dataset = new vector< SensorPoint::ConstPtr >;
    string bag_path;
    XmlRpc::XmlRpcValue bag_names, bag_labels;
    nh_priv->getParam("bag_path", bag_path);
    nh_priv->getParam("bag_names", bag_names);
    nh_priv->getParam("bag_labels", bag_labels);
    map<int, int> labels_dict;
    for(int i=0;i<bag_names.size();i++) {
      string file_loc = bag_path + (string) bag_names[i];
      loadDataBag(file_loc, bag_labels[i]);
      labels_dict[bag_labels[i]] = 0;
    }
    num_classes = labels_dict.size();
    printf("%d\n", num_classes);
  }

  void SVMOneClass::loadDataBag(string& data_bag, int label) {
    // load dataset
    rosbag::Bag bag(data_bag);
    rosbag::View view(bag, rosbag::TopicQuery("/collision_data"));
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      SensorPoint::Ptr sp = m.instantiate<SensorPoint>();
      if(sp != NULL) {
        sp->label = label;

	// remove fingertip data
	int ind = 340;
	for(int i =420;i<920;i++)
	  sp->features[ind++] = (sp->features[i]);
	for(int i =1000;i<(int) sp->features.size();i++)
	  sp->features[ind++] = (sp->features[i]);
	sp->features.resize(sp->features.size() - 160);

        dataset->push_back(sp);
      }
    }
    assert(dataset->size() != 0);
  }

  SVMOneClass::~SVMOneClass() {
  }

  void SVMOneClass::generateModel() {
    loadDataset();
    //dataset->resize(100);
    uint32_t num_one_class = 0;
    for(uint32_t i=0;i<dataset->size();i++) 
      if(dataset->at(i)->label == 0)
        num_one_class++;
    num_one_class = 4000;
    svm_problem* prob = new svm_problem;
    prob->l = num_one_class;
    prob->x = new svm_node*[num_one_class];
    prob->y = new double[num_one_class];
    uint32_t num_feats = dataset->at(0)->features.size();
    //num_feats = 30;
    int ind = 0;
    for(uint32_t i=0;i<dataset->size();i++) {
      if(dataset->at(i)->label != 0)
        continue;
      if(ind == num_one_class)
        break;
      prob->y[ind] = 1.0;
      prob->x[ind] = new svm_node[num_feats+1];
      for(uint32_t j=0;j<num_feats;j++) {
        prob->x[ind][j].index = j;
        prob->x[ind][j].value = dataset->at(i)->features[j];
      }
      prob->x[ind++][num_feats].index = -1;
    }
    printf("%d %d\n", num_one_class, ind);
    //prob->y[0] = -1.0;
    //prob->y[5] = -1.0;
    //prob->y[3] = -1.0;

    svm_parameter param;
    param.svm_type = ONE_CLASS;
    param.kernel_type = RBF;
    param.degree = 3;
    param.gamma = 0.001 / num_feats;  // 1/num_features
    param.coef0 = 0;
    param.nu = 0.98;
    param.cache_size = 8000;
    param.C = 1;
    param.eps = 1e-3;
    param.p = 0.1;
    param.shrinking = 1;
    param.probability = 0;
    param.nr_weight = 0;
    param.weight_label = new int[1];
    param.weight_label[0] = 0;
    param.weight = new double[1];
    param.weight[0] = 1;

    ROS_INFO("Train in");
    svm_model* model = svm_train(prob, &param);
    ROS_INFO("Train out");
    int cm[2][2];
    cm[0][0] = 0; cm[1][0] = 0; cm[0][1] = 0; cm[1][1] = 0; 
    for(uint32_t i=0;i<dataset->size();i++) {
      svm_node* inst = new svm_node[num_feats+1];
      for(uint32_t j=0;j<num_feats;j++) {
        inst[j].index = j;
        inst[j].value = dataset->at(i)->features[j];
      }
      inst[num_feats].index = -1;
      int pred = (int) svm_predict(model, inst);
      cm[dataset->at(i)->label != 0][pred > 0]++;
      delete[] inst;
    }
    printf("%4d %4d\n", cm[1][1], cm[0][1]);
    printf("%4d %4d\n", cm[1][0], cm[0][0]);
    /*double* target = new double[dataset->size()];
    svm_cross_validation(prob, &param, 2, target);
    for(uint32_t i=0;i<dataset->size();i++) 
      printf("%f, ", target[i]); */
    ROS_INFO("Test out");
  }

  void SVMOneClass::onInit() {
    nh = new ros::NodeHandle;
    nh_priv = new ros::NodeHandle("~");
    
    std::string results_topic, classify_topic, data_bag, forest_bag, loaded_topic;
    bool training_mode, is_validation, is_data_summary;

    nh_priv->getParam("is_validation", is_validation);
    if(is_validation) {
      return;
    }

    nh_priv->getParam("training_mode", training_mode);
    if(training_mode) {
      generateModel();
      return;
    }

    /*
    nh_priv->getParam("classify_topic", classify_topic);
    nh_priv->getParam("results_topic", results_topic);
    nh_priv->getParam("loaded_topic", loaded_topic);
    nh_priv->getParam("classifier_id", classifier_id);
    nh_priv->getParam("classifier_name", classifier_name);

    classify_sub = nh->subscribe(classify_topic.c_str(), 2, 
                        &SVMOneClass::classifyCallback, this);
    ROS_INFO("[mahalanobis_dist] Subscribed to %s", classify_topic.c_str());
    results_pub = nh->advertise<Float32>(results_topic, 1);
    ROS_INFO("[mahalanobis_dist] Publishing on %s", results_topic.c_str());
    loaded_pub = nh->advertise<Bool>(loaded_topic, 1);
    ROS_INFO("[mahalanobis_dist] Publishing on %s", loaded_topic.c_str());

    classifier_loaded = false;
    loadCovMat();
    */
  }

}

using namespace collision_detection;

void INTHandler(int sig);

void INTHandler(int sig) {
  char c;
  signal(sig, SIG_IGN);
  printf("Want to exit?\n");
  c = getchar();
  if(c == 'y' || c == 'Y')
    exit(0);
  else
    signal(SIGINT, INTHandler);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mahalanobis_dist", ros::init_options::AnonymousName);
  //signal(SIGINT, INTHandler);

  SVMOneClass md;
  md.onInit();
  ros::spin();
  printf("Exiting\n");
  return 0;
}
