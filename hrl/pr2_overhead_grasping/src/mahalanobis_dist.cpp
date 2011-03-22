#include "pr2_overhead_grasping/mahalanobis_dist.h"
//#include <pluginlib/class_list_macros.h>
#include <omp.h>
#include <stdio.h>
#include <signal.h>
//PLUGINLIB_DECLARE_CLASS(collision_detection, mahalanobis_dist, collision_detection::MahalanobisDist, nodelet::Nodelet)

using namespace std;
using namespace pr2_overhead_grasping;
using namespace std_msgs;


namespace collision_detection {

  void MahalanobisDist::loadDataset() {
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

  void MahalanobisDist::loadDataBag(string& data_bag, int label) {
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

  MahalanobisDist::~MahalanobisDist() {
  }

  void MahalanobisDist::loadCovMat() {
    string mahal_bag_name;
    string bag_path;
    double eigen_thresh;
    nh_priv->getParam("bag_path", bag_path);
    nh_priv->getParam("mahal_bag_name", mahal_bag_name);
    nh_priv->getParam("eigen_thresh", eigen_thresh);
    string file_loc = bag_path + mahal_bag_name;
    rosbag::Bag bag(file_loc);
    rosbag::View view(bag, rosbag::TopicQuery("matrix"));
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      CovarianceMatrix::Ptr cov_mat_msg = m.instantiate<CovarianceMatrix>();
      if(cov_mat_msg != NULL) {
        ROS_INFO("size %d, %d, %d", cov_mat_msg->size, cov_mat_msg->cov_mat.size(), cov_mat_msg->means.size());
        MatrixXd cov_mat(cov_mat_msg->size, cov_mat_msg->size);
        VectorXd means(cov_mat_msg->size);
        int m_ind = 0;
        for(uint32_t i=0;i<cov_mat_msg->size;i++) {
          for(uint32_t j=0;j<cov_mat_msg->size;j++)
            cov_mat(i,j) = cov_mat_msg->cov_mat[m_ind++];
          means(i) = cov_mat_msg->means[i];
        }
        ROS_INFO("size %d", cov_mat_msg->size);
        cov_inv.makeInv(cov_mat, means, eigen_thresh);
        ROS_INFO("size %d", cov_mat_msg->size);
      }
    }
    std_msgs::Bool loaded;
    loaded.data = true;
    loaded_pub.publish(loaded);
    classifier_loaded = true;
    ROS_INFO("[mahalanobis_dist] Classifier loaded.");
  }

  void MahalanobisDist::createCovMat() {
    loadDataset();
    dataset->resize(10000);
    int num_feats = dataset->at(0)->features.size();

    VectorXd means = VectorXd::Zero(num_feats);
    for(uint32_t i=0;i<dataset->size();i++) {
      for(int j=0;j<num_feats;j++) {
        means(j) += dataset->at(i)->features[j];
      }
    }
    means /= dataset->size();
    int j=0, l=0, i=0, data_size = dataset->size();
    MatrixXd var_mat = MatrixXd::Zero(num_feats, num_feats);
#pragma omp parallel default(shared) private(j, l, i) num_threads(10)
    {
      for(i=0;i<data_size;i++) {
        ROS_INFO("sample pt %d", i);
#pragma omp for schedule(dynamic, 10)
        for(j=0;j<num_feats;j++) {
          for(l=0;l<num_feats;l++) {
              var_mat(j,l) += (dataset->at(i)->features[j] - means(j)) * (dataset->at(i)->features[l] - means(l));
          }
        }
      }
    }
    var_mat /= dataset->size();
    //cov_inv[k].makeInv(var_mat, means[k], 0.01);
    saveCovMat(var_mat, means);
  }

  void MahalanobisDist::saveCovMat(MatrixXd& var_mat, VectorXd& means) {
    CovarianceMatrix cov_mat_msg;
    int num_feats = var_mat.cols();
    cov_mat_msg.size = num_feats;
    for(int j=0;j<num_feats;j++) {
      for(int l=0;l<num_feats;l++) 
        cov_mat_msg.cov_mat.push_back(var_mat(j,l));
      cov_mat_msg.means.push_back(means(j));
    }
    // save to file
    string mahal_bag_name;
    string bag_path;
    nh_priv->getParam("bag_path", bag_path);
    nh_priv->getParam("mahal_bag_name", mahal_bag_name);
    string file_loc = bag_path + mahal_bag_name;
    rosbag::Bag bag;
    string forest_bag_name;
    int bagmode = rosbag::bagmode::Write;
    bag.open(file_loc, bagmode);
    ROS_INFO("Writing covariance matrix to bag");
    bag.write("matrix", ros::Time::now(), cov_mat_msg);
    bag.close();
    ROS_INFO("Finished writing");
  }

  void MahalanobisDist::classifyCallback(const boost::shared_ptr<SensorPoint>& inst) {
    if(!classifier_loaded) {
      ROS_INFO("[mahalanobis_dist] Classifcation requested but classifier not loaded.");
      return;
    }
    VectorXd pt(inst->features.size() - 160);
    // remove fingertip data
    int ind = 0;
    for(int i =0;i<340;i++)
      pt(ind++) = inst->features[i];
    for(int i =420;i<920;i++)
      pt(ind++) = inst->features[i];
    for(int i =1000;i<(int) inst->features.size();i++)
      pt(ind++) = inst->features[i];

    Float32 dist;
    dist.data = (float) cov_inv.dist(pt);
    results_pub.publish(dist);
  }

  double MahalanobisDist::mahalanobisDist(MatrixXd& cov_inv, VectorXd& means, VectorXd& pt) {
    return (pt - means).dot(cov_inv * (pt - means));
  }

  void MahalanobisDist::makeInv(MatrixXd& A, MatrixXd& A_inv, double min_eig_val) {
    SelfAdjointEigenSolver<MatrixXd> solver(A);
    VectorXd eigenvals = solver.eigenvalues();
    ROS_INFO("MINIMUM EIGEN %f\n", eigenvals.minCoeff());
    MatrixXd eigenvectors = solver.eigenvectors();
    double thresh = eigenvals.maxCoeff() * min_eig_val;
    for(uint32_t i=0;i<eigenvals.size();i++) {
      if(eigenvals(i) < thresh)
        eigenvals(i) = 0;
      else
        eigenvals(i) = 1.0 / eigenvals(i);
    }
    A_inv = eigenvectors * eigenvals.asDiagonal() * eigenvectors.transpose();

    SelfAdjointEigenSolver<MatrixXd> solver2(A_inv);
    ROS_INFO("MINIMUM EIGEN2 %f\n", solver2.eigenvalues().minCoeff());
  }

  void MahalanobisDist::doMahalanobis() {
    loadDataset();
    vector<vector< SensorPoint::ConstPtr > > datasets(1000); // split by label
    for(uint32_t i=0;i<dataset->size();i++) 
      datasets[dataset->at(i)->label].push_back(dataset->at(i));
    for(uint32_t i=0;i<datasets.size();i++) 
      if(datasets[i].size() == 0) {
        datasets.resize(i);
        num_classes = i;
        break;
      }
    datasets[0].resize(10000);
    datasets[1].resize(10000);
    int num_feats = datasets[0][0]->features.size();

    vector<DistFinder> cov_inv(num_classes);

    vector<VectorXd> means(num_classes);
    for(int k=0;k<num_classes;k++) {
      means[k] = VectorXd::Zero(num_feats);
      for(uint32_t i=0;i<datasets[k].size();i++) {
        for(int j=0;j<num_feats;j++) {
          means[k](j) += datasets[k][i]->features[j];
        }
      }
      means[k] /= datasets[k].size();
    }
    int k=0, j=0, l=0, i=0, data_size = datasets[k].size();
    //for(k=0;k<num_classes;k++) {
    for(k=0;k<1;k++) {
      //ROS_INFO("k %d, num_classes %d", j, num_classes);
      MatrixXd var_mat = MatrixXd::Zero(num_feats, num_feats);
#pragma omp parallel default(shared) private(j, l, i) num_threads(10)
    {
      for(i=0;i<data_size;i++) {
        ROS_INFO("sample pt %d", i);
#pragma omp for schedule(dynamic, 10)
        for(j=0;j<num_feats;j++) {
          for(l=0;l<num_feats;l++) {
            //if(l != j) continue;
              var_mat(j,l) += (datasets[k][i]->features[j] - means[k](j)) * (datasets[k][i]->features[l] - means[k](l));
          }
        }
      }
    }
      var_mat /= datasets[k].size();
      //double sum = 0.0;
      //  for(j=0;j<num_feats;j++) 
      //    for(l=0;l<num_feats;l++) 
      //      sum += var_mat(j,l) - var_mat(l,j);
      //cout << sum << endl;

      cov_inv[k].makeInv(var_mat, means[k], 0.000001);
    }
    //for(int k=0;k<num_classes;k++) {
    for(int k=0;k<1;k++) {
      for(int l=0;l<num_classes;l++) {
        ArrayXd dists(datasets[l].size());
        int ind = 0, data_size = datasets[l].size();
        vector<VectorXd> feat_vecs(data_size);
        for(ind=0;ind<data_size;ind++) {
          feat_vecs[ind] = VectorXd::Zero(num_feats);
          for(j=0;j<num_feats;j++) 
            feat_vecs[ind](j) = datasets[l][ind]->features[j];
        }
#pragma omp parallel default(shared) private(ind) num_threads(10)
    {
#pragma omp for schedule(dynamic, 10)
        for(ind=0;ind<data_size;ind++) {
          //dists[ind] = mahalanobisDist(cov_inv[k], means[k], feat_vec);
          dists[ind] = cov_inv[k].dist(feat_vecs[ind]);
        }
    }
        
        int nans = 0, negs = 0;
        for(int i=0;i<data_size;i++) {
          if(dists[i] < 0)
            negs++;
          if(dists[i] != dists[i]) {
            dists[i] = 0;
            nans++;
          }
        }
        //cout << dists << endl;
        double mean_dist = dists.sum() / (datasets[l].size() - nans);
        VectorXd diff = dists - mean_dist;
        double var_dist = diff.dot(diff) / (datasets[l].size() - nans);
        double std_dist = sqrt(var_dist);
        double min_dist = dists.minCoeff();
        double max_dist = dists.maxCoeff();
        printf("cov %d, data %d, mean_dist %e, std_dist %e, min %e, max %e, nans %d, negs %d\n", k, l, mean_dist, std_dist, min_dist, max_dist, nans, negs);
      }
      printf("cov %d, num_samps %d, rank 0\n", k, (int) datasets[k].size()); //, cov_inv[k]->rank());
    }
    printf("num_classes %d, num_feats %d\n", num_classes, num_feats);
  }

  void MahalanobisDist::summarizeData() {
    loadDataset();
    int num_feats = dataset->at(0)->features.size();

    MatrixXd means = MatrixXd::Zero(num_classes, num_feats);
    MatrixXd vars = MatrixXd::Zero(num_classes, num_feats);
    MatrixXd mins = MatrixXd::Zero(num_classes, num_feats);
    MatrixXd maxs = MatrixXd::Zero(num_classes, num_feats);
    VectorXi nums = VectorXi::Zero(num_classes);
    for(int l=0;l<num_classes;l++) 
      for(uint32_t i=0;i<dataset->size();i++)
        nums(dataset->at(i)->label)++;

    for(int l=0;l<num_classes;l++) {
      MatrixXd data = MatrixXd::Zero(nums(l), num_feats);
      int ind = 0;
      for(uint32_t i=0;i<dataset->size();i++) {
        if(dataset->at(i)->label != l)
          continue;
        for(int j=0;j<num_feats;j++) 
          data(ind,j) = abs(dataset->at(i)->features[j]);
        ind++;
      }
      nums(l) = ind;
      for(int j=0;j<num_feats;j++) {
        double mean = data.col(j).sum() / ind;
        VectorXd err = data.col(j).array() - mean;
        double var = err.dot(err) / ind;
        double min = data.col(j).minCoeff();
        double max = data.col(j).maxCoeff();

        means(l,j) = mean; vars(l, j) = var; mins(l, j) = min; maxs(l, j) = max;
      }
    }
    
    for(int j=0;j<num_feats;j++) {
      printf("\n%d\n", j);
      for(int l=0;l<num_classes;l++) 
        printf("%+1.3e %+1.3e %+1.3e %+1.3e\n", means(l,j), vars(l,j), mins(l,j), maxs(l,j));
    }
    cout << nums;
  }

  void MahalanobisDist::onInit() {
    nh = new ros::NodeHandle;
    nh_priv = new ros::NodeHandle("~");
    
    std::string results_topic, classify_topic, data_bag, forest_bag, loaded_topic;
    bool training_mode, is_validation, is_data_summary;

    nh_priv->getParam("is_validation", is_validation);
    if(is_validation) {
      doMahalanobis();
      return;
    }

    nh_priv->getParam("training_mode", training_mode);
    if(training_mode) {
      createCovMat();
      return;
    }

    nh_priv->getParam("is_data_summary", is_data_summary);
    if(is_data_summary) {
      summarizeData();
      return;
    }

    nh_priv->getParam("classify_topic", classify_topic);
    nh_priv->getParam("results_topic", results_topic);
    nh_priv->getParam("loaded_topic", loaded_topic);
    nh_priv->getParam("classifier_id", classifier_id);
    nh_priv->getParam("classifier_name", classifier_name);

    classify_sub = nh->subscribe(classify_topic.c_str(), 2, 
                        &MahalanobisDist::classifyCallback, this);
    ROS_INFO("[mahalanobis_dist] Subscribed to %s", classify_topic.c_str());
    results_pub = nh->advertise<Float32>(results_topic, 1);
    ROS_INFO("[mahalanobis_dist] Publishing on %s", results_topic.c_str());
    loaded_pub = nh->advertise<Bool>(loaded_topic, 1);
    ROS_INFO("[mahalanobis_dist] Publishing on %s", loaded_topic.c_str());

    classifier_loaded = false;
    loadCovMat();
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

  MahalanobisDist md;
  md.onInit();
  ros::spin();
  printf("Exiting\n");
  return 0;
}
