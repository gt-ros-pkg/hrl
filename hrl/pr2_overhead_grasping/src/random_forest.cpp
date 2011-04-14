#include "pr2_overhead_grasping/random_forest.h"
//#include <pluginlib/class_list_macros.h>
#include <omp.h>
#include <stdio.h>
#include <signal.h>
//PLUGINLIB_DECLARE_CLASS(collision_detection, random_forest, collision_detection::RandomForest, nodelet::Nodelet)

using namespace std;
using namespace pr2_overhead_grasping;
int ATTRS_TRY = 44;
double MAX_GAIN_THRESH = 0.0;

namespace collision_detection {

  RandomTree::RandomTree(int c_d_tree_num) {
    d_tree_num = c_d_tree_num;
  }

  RandomTree::RandomTree(RandomTreeMsg::Ptr r_tree) {
    rand_tree = r_tree;
    num_classes = rand_tree->num_classes;
  }

  bool RandomTree::attrCompare(int inst_i, int inst_j, int attr) { 
    return (dataset->at(inst_i)->features[attr] < dataset->at(inst_j)->features[attr]);
  }

  struct AttrComp {
    int attr;
    vector< SensorPoint::Ptr >* dataset;
    AttrComp(vector< SensorPoint::Ptr >* c_dataset, int c_attr) { 
      dataset = c_dataset;
      attr = c_attr;
    }
    bool operator()(int inst_i, int inst_j) {
      return (dataset->at(inst_i)->features[attr] < dataset->at(inst_j)->features[attr]);
    }
  };

  typedef set<int, boost::function<bool (int,int)> > AttrSet;
  void RandomTree::findBestSplit(vector<int>* insts, vector<int>& attrs,
                                             pair<int, float>& ret) {
    long double max_gain = 0.0;
    float best_split_f = 0;
    int best_split_attr = 0;
    for(uint32_t k=0;k<insts->size();k++)
      if((uint32_t) insts->at(k) >= dataset->size())
        printf("WTF\n");
    for(uint32_t a=0;a<attrs.size();a++) {
      //cout << "0-----------------------------------------------------------" << endl;
      //boost::function<bool (int,int)> attr_comp = boost::bind(&RandomTree::attrCompare, *this, _1, _2, attrs[a]);
      AttrComp attr_comp(dataset, attrs[a]);
      //cout << attrs[a] << " " << dataset->at(0)->features.size() << endl;
      //AttrSet insts_sorted(attr_comp);
      vector<int> insts_temp(insts->begin(), insts->end());
      set<int, AttrComp> insts_sorted(attr_comp);
      //cout << "0-----------------------------------------------------------" << endl;
      insts_sorted.insert(insts_temp.begin(), insts_temp.end()); 
      //cout << "0-----------------------------------------------------------" << endl;
      map<int, int> class_sums;
      map<int, int> class_cur_sums;
      map<int, int>::iterator cs_iter, ccs_iter;
      //cout << "1-----------------------------------------------------------" << endl;
      for(uint32_t i =0;i<insts->size();i++) {
        int label = dataset->at(insts->at(i))->label;
        if(class_sums.count(label) == 0) {
          class_sums[label] = 0;
          class_cur_sums[label] = 0;
        }
        class_sums[label]++;
      }
      //class_cur_sums[labels[insts_sorted[0]]]++;
      //cout << "2-----------------------------------------------------------" << endl;
      uint32_t inst_ctr = 0;
      for(AttrSet::iterator inst = insts_sorted.begin();
                                              inst != insts_sorted.end();inst++) {
        if(inst_ctr == insts_sorted.size()-1)
          break;
        //cout << dataset->at(insts_sorted[i])->features[attrs[a]] << endl;
        class_cur_sums[dataset->at(*inst)->label]++;
        long double entropy = 0.0, gain = 0.0;
        cs_iter = class_sums.begin();
        ccs_iter = class_cur_sums.begin();
        for(int j=0;j<num_classes;j++) { // find entropy
          long double prec_in_class_l = ccs_iter->second / (long double) (insts->size());
          if(prec_in_class_l > 0.00001)
            entropy -= prec_in_class_l * log2(prec_in_class_l);
          long double prec_in_class_r = (cs_iter->second - ccs_iter->second) / (long double) (insts->size());
          if(prec_in_class_r > 0.00001)
            entropy -= prec_in_class_r * log2(prec_in_class_r);
          //cout << prec_in_class_l << ", " << prec_in_class_r << ", ";
          gain += entropy * cs_iter->second;
          cs_iter++; ccs_iter++;
        }
        //cout << "3-----------------------------------------------------------" << endl;
        //cout << entropy << ", " << i << endl;
        if(gain > max_gain) { // find best entropy
          best_split_f = dataset->at(*inst)->features[attrs[a]]; // split point x <= best_split_f : left
          best_split_attr = attrs[a]; // attribute being compared
          //cout << *inst << ", " << insts_sorted.size() << ", " << dataset->at(*inst)->features[attrs[a]] << ", " << dataset->at(*inst)->features[attrs[a]] << ", " << *inst << ", " << *inst << ", " << gain << ", " << class_sums[0] << ", " << class_sums[1] << ", " << class_cur_sums[0] << ", " << class_cur_sums[1] << endl;
          max_gain = gain;
          //if(gain > MAX_GAIN_THRESH) 
            // return and say that this is a terminal split
            //return make_pair(-best_split_attr, best_split_f);
        }
        //cout << "4-----------------------------------------------------------" << endl;
        inst_ctr++;
      }
    }
    ret.first = best_split_attr; ret.second =  best_split_f;
  }

  void RandomTree::splitNode(vector<int>* node_inds, 
                             pair<int, float>& split_pt,
                             pair<vector<int>*, vector<int>* >& split_nodes) {
    pair<vector<int>*, vector<int>* > nodes;
    split_nodes.first = new vector<int>; split_nodes.second = new vector<int>;
    //cout << "YOOOOOOOO" << endl;
    for(uint32_t i=0;i<node_inds->size();i++) {
      //cout << dataset->at(node_inds->at(i))->features[split_pt.first] << endl;
      if(dataset->at(node_inds->at(i))->features[split_pt.first] <= split_pt.second)
        split_nodes.first->push_back(node_inds->at(i));
      else
        split_nodes.second->push_back(node_inds->at(i));
    }
    assert(split_nodes.first->size() > 0);
    assert(split_nodes.second->size() > 0);
  }

  void RandomTree::growTree(vector< SensorPoint::Ptr >* c_dataset,
                            vector<int>* inds) {
    ////////////////////////////////////////////////////////////////////////////////
    // Balanced Random Tree
    // Taken from "Using Random Forest to Learn Imbalanced Data"
    // Chao Chen, Andy Liaw, Leo Breiman
    // 2004
    ////////////////////////////////////////////////////////////////////////////////
    dataset = c_dataset;
    //cout << "YO 1" << endl;
    int num_attrs = dataset->at(0)->features.size();

    // Tree will be saved as this message
    rand_tree = boost::shared_ptr<RandomTreeMsg>(new RandomTreeMsg);
    //cout << "YO 2" << endl;
    rand_tree->tree_num = d_tree_num;
    rand_tree->attr_split.resize(2*inds->size());
    //cout << "YO 3" << endl;
    rand_tree->val_split.resize(2*inds->size());
    rand_tree->r_node_inds.resize(2*inds->size(), -999);
    vector<int> r_node_stack;
    //cout << "YO 4" << endl;
    int tm_i = 0;

    ////////////////////////////////////////////////////////////////////////////////
    // make samples
    map<int, vector<int> > class_inds; // list of indices indexed by label
    for(uint32_t i=0;i<inds->size();i++) 
      class_inds[dataset->at(inds->at(i))->label].push_back(inds->at(i));
    num_classes = class_inds.size();
    // find smallest class in the sample
    int minority_class = 0, min_class = class_inds.begin()->second.size(); 
    for(map<int, vector<int> >::iterator i=class_inds.begin();i!=class_inds.end();i++) {
      if(i->second.size() < (uint32_t) min_class) {
        min_class = i->second.size();
        minority_class = i->first;
      }
    }
    //cout << "YO 5" << endl;
    vector<bool> oobs(dataset->size(), true);
    // Balance the sample sets
    int num_samples = class_inds[minority_class].size();
    // sampled top node to split
    vector<int>* head_node = new vector<int>(num_samples*class_inds.size()); 
    int i_cntr = 0;
    for(map<int, vector<int> >::iterator i=class_inds.begin();i!=class_inds.end();i++) {
      //cout << head_node->size() << " Size" << endl;
      for(int j=0;j<num_samples;j++) {
        int sample_ind = i->second[rand() % i->second.size()];
        //cout << sample_ind << endl;
        head_node->at(i_cntr++) = sample_ind;
        oobs[sample_ind] = false;
      }
    }
    //cout << "YO 6" << endl;
    /* Unbalanced tree
    int num_samples = dataset->size();
    for(uint32_t i=0;i<num_samples;i++) {
      int sample_ind = rand() % dataset->size();
      head_node->push_back(sample_ind);
      oobs[sample_ind] = false;
    }*/
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////
    // Create tree
    int last_r_node_ind = -1;
    vector<vector<int>* > node_stack;
    node_stack.push_back(head_node);
    //cout << "YO 7" << endl;
    while(ros::ok()) {
      if(node_stack.size() == 0)
        break;
      vector<int>* cur_node = node_stack.back(); 
      //cout << node_stack.size() << endl;

      bool homo = true; // is the node homogenous?
      for(uint32_t i=0;i<cur_node->size();i++) {
        //cout << dataset->at(cur_node->at(0))->label << ", " << dataset->at(cur_node->at(i))->label << endl;
        if(dataset->at(cur_node->at(0))->label != dataset->at(cur_node->at(i))->label) {
          homo = false;
          break;
        }
      }
      if(homo) {
        // terminal: everything of same class
        // note that negative values indicate sentinels
        // the class of that terminal node is the absolute value of the first value

        // add terminal node to tree
        //cout << dataset->size() << endl;
        //cout << cur_node->size() << endl;
        //cout << cur_node->at(0) << endl;
        rand_tree->attr_split[tm_i] = dataset->at(cur_node->at(0))->label;
        rand_tree->val_split[tm_i] = 0.0;
        rand_tree->r_node_inds[tm_i] = -1; // this node is terminal
        // the parent of this node's right node is the next element
        if(r_node_stack.size() > 0) {
          last_r_node_ind = r_node_stack.back();
          rand_tree->r_node_inds[r_node_stack.back()] = tm_i + 1; 
          r_node_stack.pop_back();
          tm_i++;
        }

        node_stack.pop_back();
        //cout << "YO in 0" << endl;
        //delete cur_node;
        //cout << "YO ot 0" << endl;
        continue;
      }
      assert(cur_node->size() > 1);

      // get a random selection of attributes to check for a split
      int num_try = (int) sqrt(num_attrs);
      vector<int> attrs(num_try, 0); 
      for(int i=0;i<num_try;i++) {
        attrs[i] = rand() % num_attrs;
      }

      // find a splitting point on this node
      // <attribute to split on, value to split at> if x <= v :-> go left
      //cout << cur_node->size() << "Size in" << endl;
      pair<int, float> split_pt;
      findBestSplit(cur_node, attrs, split_pt);
      //cout << cur_node->size() << "Size out" << endl;
      if(split_pt.first < 0) {
        // DISABLED
        assert(false);
        // terminal, entropy is small
        split_pt.first *= -1;
        // left and right split nodes
        pair<vector<int>*, vector<int>* > new_nodes;
        splitNode(cur_node, split_pt, new_nodes);

        // left
        vector<int> class_sums_l(num_classes, 0);
        for(uint32_t i=0;i<new_nodes.first->size();i++) 
          class_sums_l[dataset->at(new_nodes.first->at(i))->label]++;
        int popular_class_l = max_element(class_sums_l.begin(), class_sums_l.end()) - class_sums_l.begin();

        // add terminal left node to tree
        rand_tree->attr_split[tm_i] = popular_class_l;
        rand_tree->val_split[tm_i] = 0.0;
        rand_tree->r_node_inds[tm_i] = -1; // this node is terminal
        // the parent of this node's right node is the next element
        rand_tree->r_node_inds[r_node_stack.back()] = tm_i + 1; 
        r_node_stack.pop_back();
        tm_i++;

        // right
        vector<int> class_sums_r(num_classes, 0);
        for(uint32_t i=0;i<new_nodes.second->size();i++) 
          class_sums_r[dataset->at(new_nodes.second->at(i))->label]++;
        int popular_class_r = max_element(class_sums_r.begin(), class_sums_r.end()) - class_sums_r.begin();

        // add terminal right node to tree
        rand_tree->attr_split[tm_i] = popular_class_r;
        rand_tree->val_split[tm_i] = 0.0;
        rand_tree->r_node_inds[tm_i] = -1; // this node is terminal
        // the parent of this node's right node is the next element
        rand_tree->r_node_inds[r_node_stack.back()] = tm_i + 1; 
        r_node_stack.pop_back();
        tm_i++;
      } else {
        // recurse upon both left and right nodes
        // left and right split nodes
        pair<vector<int>*, vector<int>* > new_nodes;
        splitNode(cur_node, split_pt, new_nodes);

        node_stack.pop_back();
        // normal
        node_stack.push_back(new_nodes.second); 
        // put the left node at the top of the stack
        node_stack.push_back(new_nodes.first); 

        // add split to tree
        rand_tree->attr_split[tm_i] = split_pt.first;
        rand_tree->val_split[tm_i] = split_pt.second;
        r_node_stack.push_back(tm_i);
        tm_i++;
      }
      //cout << "YO in 1" << endl;
      //delete cur_node;
      //cout << "YO ot 1" << endl;
    }
    //cout << "YO 8" << endl;
    rand_tree->r_node_inds[last_r_node_ind] = tm_i - 1; 
    //cout << r_node_stack[0] << ", " << rand_tree->r_node_inds[0] << endl;
    //exit(1);

    // we have an pre-order tree stored at d_tree
    // resize message back to minimum size
    rand_tree->attr_split.resize(tm_i);
    rand_tree->val_split.resize(tm_i);
    rand_tree->r_node_inds.resize(tm_i);
    //cout << tm_i << endl;
    for(uint32_t i=0;i<oobs.size();i++)
      if(oobs[i])
        rand_tree->out_of_bags.push_back(i);
    rand_tree->num_classes = num_classes;
    // rand_tree is fully built
    //cout << "YO TREE OUT" << endl;
  }

  void RandomTree::writeTree(string& bag_file, bool is_first) {
    // save tree to file
    rosbag::Bag bag;
    int bagmode;
    if(is_first)
      bagmode = rosbag::bagmode::Write;
    else
      bagmode = rosbag::bagmode::Append;
    bag.open(bag_file, bagmode);
    bag.write("trees", ros::Time::now(), rand_tree);
    bag.close();
  }

  int RandomTree::classifyInstance(SensorPoint::Ptr inst) {
    int ind = 0, attr;

    //cout << rand_tree->attr_split.size() << ", " << rand_tree->r_node_inds.size() << ", " << rand_tree->val_split.size() << endl;
    //for(int i=0;i<rand_tree->attr_split.size();i++)
    //  cout << i << ", " << rand_tree->attr_split[i] << ", " << rand_tree->val_split[i] << ", " << rand_tree->r_node_inds[i] << ", " << rand_tree->r_node_inds[rand_tree->r_node_inds[i]]  << endl;
      

    //exit(1);
    //cout << "Class in" << endl;
    while(ros::ok()) {
      //cout << attr << ", " << ind << ", " << rand_tree->r_node_inds[ind] << ", " << inst->features.size() << ", " << rand_tree->r_node_inds.size() <<", " << rand_tree->val_split.size() << ", " << rand_tree->attr_split.size() << endl;
      attr = rand_tree->attr_split[ind];
      assert(attr >= 0 && attr < inst->features.size());
      //cout << ind << " ";
      assert(ind != -999);
      if(rand_tree->r_node_inds[ind] < 0) {
        // terminal node
        //cout << "Class out" << endl;
        return attr;
      }
      float feat = inst->features[attr];
      if(is_abs)
        feat = abs(feat);
      if(feat <= rand_tree->val_split[ind]) {
        // go left
        ind++;
      } else {
        // go right
        ind = rand_tree->r_node_inds[ind];
      }
    }
    //NODELET_FATAL("BAD RANDOM TREE");
    return -1;
  }

  void RandomForest::loadDataset() {
    dataset = new vector< SensorPoint::Ptr >;
    string bag_path;
    XmlRpc::XmlRpcValue bag_names, bag_labels;
    nh_priv->getParam("bag_path", bag_path);
    nh_priv->getParam("bag_names", bag_names);
    nh_priv->getParam("bag_labels", bag_labels);
    for(int i=0;i<bag_names.size();i++) {
      string file_loc = bag_path + (string) bag_names[i];
      loadDataBag(file_loc, bag_labels[i]);
    }
  }

  void RandomForest::loadDataBag(string& data_bag, int label) {
    // load dataset
    rosbag::Bag bag(data_bag);
    rosbag::View view(bag, rosbag::TopicQuery("/collision_data"));
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      SensorPoint::Ptr sp = m.instantiate<SensorPoint>();
      if(sp != NULL) {
        sp->label = label;
        if(is_abs)
          for(uint32_t i=0;i<sp->features.size();i++)
            sp->features[i] = abs(sp->features[i]);
        dataset->push_back(sp);
      }
    }
    assert(dataset->size() != 0);
  }

  void RandomForest::setDataset(vector< SensorPoint::Ptr >* datas) {
    dataset = datas;
  }

  void RandomForest::growForest(vector< SensorPoint::Ptr >* c_dataset,
                            vector<int>* inds, int c_num_trees) {
    dataset = c_dataset;
    num_trees = c_num_trees;
    // grow trees
    trees = new RandomTree*[num_trees];
    int i, NUM_CHUNKS = 10;
    RandomForest* this_forest = this;
#pragma omp parallel shared(this_forest, inds, NUM_CHUNKS) private(i)
    {
#pragma omp for schedule(dynamic, NUM_CHUNKS)
      for(i=0;i<this_forest->num_trees;i++) {
        ROS_INFO("Growing tree %d", i+1);
        //cout << i << " " << omp_get_thread_num() << " " << omp_get_num_threads() << endl;
        while(ros::ok()) {
          try {
            this_forest->trees[i] = new RandomTree(i);
            this_forest->trees[i]->growTree(this_forest->dataset, inds);
            break;
          }
          catch (bad_alloc &) {
            ROS_INFO("Memory unavailable...restaring...");
            ros::Duration(1.0).sleep();
          }
        }
      }
    }
  }

  RandomForest::~RandomForest() {
    for(int i=0;i<num_trees;i++)
      delete trees[i];
  }

  void RandomForest::writeForest() {
    string forest_bag_name;
    nh_priv->getParam("forest_bag_name", forest_bag_name);
    writeForest(forest_bag_name);
  }

  void RandomForest::writeForest(string name) {
    string bag_path;
    nh_priv->getParam("bag_path", bag_path);
    string file_loc = bag_path + name;
    for(int i=0;i<num_trees;i++) {
      trees[i]->writeTree(file_loc, i == 0);
    }
  }


  void RandomForest::growWriteForest() {
    int c_num_trees;
    bool is_validation;
    nh_priv->getParam("num_trees", c_num_trees);
    nh_priv->getParam("is_validation", is_validation);
    loadDataset();
    vector<int>* inds = new vector<int>(dataset->size());
    for(uint32_t i=0;i<dataset->size();i++) 
      inds->at(i) = i;
    if(is_validation) {
      int pos_id;
      bool classify_first;
      nh_priv->getParam("pos_id", pos_id);
      nh_priv->getParam("classify_first", classify_first);
      ROS_INFO("Running ten-fold cross validation.");
      vector<map<int, int> > votes_total;
      RandomForest::runTenFold(dataset, pos_id, c_num_trees, votes_total, classify_first);
      // vote filter
      bool filter;
      nh_priv->getParam("filter", filter);
      if(filter) {
        double filter_thresh;
        string filter_out, bag_path;
        ROS_INFO("Filtering bad data.");
        nh_priv->getParam("filter_thresh", filter_thresh);
        nh_priv->getParam("bag_path", bag_path);
        nh_priv->getParam("filter_out", filter_out);
        string file_loc = bag_path + filter_out;
        rosbag::Bag bag;
        int bagmode = rosbag::bagmode::Write;
        bag.open(file_loc, bagmode);
        for(uint32_t i = 0;i<votes_total.size();i++) {
          if(votes_total[i][pos_id] / (double) c_num_trees < filter_thresh &&
              dataset->at(i)->label != (uint32_t) pos_id) {
            // write
            bag.write("/collision_data", ros::Time::now(), dataset->at(i));
          }
        }
        bag.close();
      }
      ROS_INFO("Done, please exit.");
    } else {
      ROS_INFO("Growing forest");
      growForest(dataset, inds, c_num_trees);
      ROS_INFO("Writing forest");
      writeForest();
      ROS_INFO("Finished writing forest, please exit");
    }
  }

  void RandomForest::loadForest() {
    string forest_bag_name;
    string bag_path;
    nh_priv->getParam("bag_path", bag_path);
    nh_priv->getParam("forest_bag_name", forest_bag_name);
    string file_loc = bag_path + forest_bag_name;
    rosbag::Bag bag(file_loc);
    rosbag::View view(bag, rosbag::TopicQuery("trees"));
    int ind = 0;
    trees = new RandomTree*[view.size()];
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      RandomTreeMsg::Ptr tree = m.instantiate<RandomTreeMsg>();
      if(tree != NULL) {
        trees[ind++] = new RandomTree(tree);
        trees[ind-1]->is_abs = is_abs;
        oobs.push_back(tree->out_of_bags);
      }
    }
    num_trees = ind;
    num_classes = trees[0]->num_classes;
    std_msgs::Bool loaded;
    loaded.data = true;
    loaded_pub.publish(loaded);
    trees_loaded = true;
    ROS_INFO("[random_forest] All trees loaded.");
  }

  void RandomForest::loadCovMat() {
    string mahal_bag_name;
    string bag_path;
    nh_priv->getParam("bag_path", bag_path);
    nh_priv->getParam("mahal_bag_name", mahal_bag_name);
    string file_loc = bag_path + mahal_bag_name;
    rosbag::Bag bag(file_loc);
    rosbag::View view(bag, rosbag::TopicQuery("matrix"));
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      CovarianceMatrix::Ptr cov_mat_msg = m.instantiate<CovarianceMatrix>();
      if(cov_mat_msg != NULL) {
        MatrixXd cov_mat(cov_mat_msg->size, cov_mat_msg->size);
        int m_ind = 0;
        for(uint32_t i=0;i<cov_mat_msg->size;i++) {
          for(uint32_t j=0;j<cov_mat_msg->size;j++)
            cov_mat(i,j) = cov_mat_msg->cov_mat[m_ind++];
          means(i) = cov_mat_msg->means[i];
        }
        cov_inv = new LDLT<MatrixXd>(cov_mat);
      }
    }
    std_msgs::Bool loaded;
    loaded.data = true;
    loaded_pub.publish(loaded);
    trees_loaded = true;
    ROS_INFO("[random_forest] All trees loaded.");
  }

  void RandomForest::createCovMat() {
    loadDataset();
    //dataset->resize(10000);
    int num_feats = dataset->at(0)->features.size();

    VectorXd means = VectorXd::Zero(num_feats);
    for(uint32_t i=0;i<dataset->size();i++) {
      for(int j=0;j<num_feats;j++) {
        means(j) += dataset->at(i)->features[j];
      }
    }
    means /= dataset->size();
    int j=0, l=0;
    uint32_t i=0;
    MatrixXd var_mat = MatrixXd::Zero(num_feats, num_feats);
#pragma omp parallel default(shared) private(j, l, i) num_threads(10)
    {
#pragma omp for schedule(dynamic, 10)
      for(j=0;j<num_feats;j++) {
        ROS_INFO("j %d", j);
        for(l=0;l<num_feats;l++) {
          for(i=0;i<dataset->size();i++) 
            var_mat(j,l) += (dataset->at(i)->features[j] - means(j)) * (dataset->at(i)->features[l] - means(l));
          var_mat(j,l) /= dataset->size();
        }
      }
    }

    CovarianceMatrix cov_mat_msg;
    cov_mat_msg.size = num_feats;
    for(j=0;j<num_feats;j++) 
      for(l=0;l<num_feats;l++) 
        cov_mat_msg.cov_mat.push_back(var_mat(j,l));
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
    bag.write("matrix", ros::Time::now(), cov_mat_msg);
    bag.close();
  }

  void RandomForest::collectVotes(SensorPoint::Ptr inst, map<int, int>& class_votes) {
    int i;
    //cout << "IN ";
    for(i=0;i<num_trees;i++) {
      int class_vote = trees[i]->classifyInstance(inst);
      if(class_votes.count(class_vote) == 0) 
        class_votes[class_vote] = 0;
      class_votes[class_vote]++;
      //cout<< class_vote << ", ";
    }
    //cout << " OUT" << endl;
  }

  void RandomForest::classifyCallback(const boost::shared_ptr<SensorPoint>& inst) {
    if(!trees_loaded) {
      ROS_INFO("[random_forest] Classifcation requested but trees not loaded.");
      return;
    }
    ClassVotes::Ptr class_votes(new ClassVotes);
    map<int, int> cvs;
    for(uint32_t i=0;i<inst->features.size();i++)
      if(inst->features[i] != inst->features[i]) {
        ROS_INFO("NAN %d", i);
        return;
      }
    //cout << endl;
    collectVotes(inst, cvs);
    class_votes->votes.resize(num_classes);
    class_votes->classes.resize(num_classes);
    int ind = 0;
    for(map<int, int>::iterator iter=cvs.begin();iter!=cvs.end();iter++) {
      //class_votes->votes.push_back(iter->second);
      //ROS_INFO("[random_forest] %d %d", iter->first, iter->second);
      class_votes->votes[iter->first] = iter->second;
      class_votes->classes[ind++] = iter->first;
    }
    class_votes->classifier_name = classifier_name;
    class_votes->classifier_id = classifier_id;
    results_pub.publish(class_votes);
  }

  // finds the first class which is not the positive class
  int RandomForest::findFirstClass(vector<pair<map<int, int>, float > >* votes_list, int pos_id, float thresh) {
    int min_class = pos_id;
    float min_delay = 10000.0;
    for(uint32_t j=0;j<votes_list->size();j++) {
      int pred_class;
      if(votes_list->at(j).first[pos_id] > thresh) 
        pred_class = pos_id;
      else {
        int max_class = 0, max_votes = -1;
        for(map<int, int>::iterator iter=votes_list->at(j).first.begin();iter!=votes_list->at(j).first.end();iter++) {
          if(iter->second > max_votes) {
            max_class = iter->first;
            max_votes = iter->second;
          }
        }
        pred_class = max_class;
      }
      if(votes_list->at(j).second < min_delay && pred_class != pos_id) {
        min_class = pred_class;
        min_delay = votes_list->at(j).second;
      }
    }
    return min_class;
  }

  int RandomForest::findFrequentClass(vector<pair<map<int, int>, float > >* votes_list, int pos_id, float thresh) {
    // all of the vote maps in this trajectory
    map<int, int> vote_counts;
    for(uint32_t j=0;j<votes_list->size();j++) {
      int pred_class;
      if(votes_list->at(j).first[pos_id] > thresh) 
        pred_class = pos_id;
      else {
        int max_class = 0, max_votes = -1;
        for(map<int, int>::iterator iter=votes_list->at(j).first.begin();iter!=votes_list->at(j).first.end();iter++) {
          if(iter->second > max_votes) {
            max_class = iter->first;
            max_votes = iter->second;
          }
        }
        pred_class = max_class;
      }
      if(vote_counts.count(pred_class) == 0)
        vote_counts[pred_class] = 0;
      vote_counts[pred_class]++;
    }
    int max_class = 0, max_votes = -1;
    for(map<int, int>::iterator iter=vote_counts.begin();iter!=vote_counts.end();iter++) {
      if(iter->second > max_votes) {
        max_class = iter->first;
        max_votes = iter->second;
      }
    }
    return max_class;
  }

  void RandomForest::runTenFold(vector< SensorPoint::Ptr >* train_test_data, 
                                       int pos_id,
                                       int c_num_trees,
                                       vector<map<int,int> >& votes_total,
                                       bool classify_first) {
    int num_roc = 21;
    int NUM_FOLDS = 10;
    vector<int> traj_ids;
    map<int, int> label_cntr; 
    vector<int> inv_label_cntr;
    map<int, vector<int> > traj_id_map;
    int label_ind = 0;
    for(uint32_t i=0;i<train_test_data->size();i++) {
      int traj_id = train_test_data->at(i)->traj_id;
      if(traj_id_map[traj_id].size() == 0)
        traj_ids.push_back(traj_id);
      traj_id_map[traj_id].push_back(i);
      if(label_cntr.count(train_test_data->at(i)->label) == 0) {
        inv_label_cntr.push_back(train_test_data->at(i)->label);
        label_cntr[train_test_data->at(i)->label] = label_ind++;
      }
    }
    int num_classes = label_cntr.size();

    // create folds
    FoldData::Ptr fold_save(new FoldData);
    random_shuffle(traj_ids.begin(), traj_ids.end());
    vector<int>* folds[NUM_FOLDS];
    vector<int>* f_tests[NUM_FOLDS];
    for(int i=0;i<NUM_FOLDS;i++) {
      folds[i] = new vector<int>;
      f_tests[i] = new vector<int>;
    }
    for(uint32_t i=0;i<traj_ids.size();i+=NUM_FOLDS) {
      for(int j=0;j<NUM_FOLDS;j++) {
        if(i+j == traj_ids.size())
          break;
        for(int k=0;k<NUM_FOLDS;k++) {
          int id = traj_ids[i+j];
          if(k != j) {
            for(uint32_t l=0;l<traj_id_map[id].size();l++) {
              folds[k]->push_back(traj_id_map[id][l]);
              fold_save->fold_data.push_back(traj_id_map[id][l]);
            }
          } else {
            for(uint32_t l=0;l<traj_id_map[id].size();l++) {
              f_tests[k]->push_back(traj_id_map[id][l]);
              fold_save->test_data.push_back(traj_id_map[id][l]);
            }
          }
        }
      }
    } 
    /*
    for(int k=0;k<NUM_FOLDS;k++) {
      fold_save->fold_sizes.push_back(folds[k]->size());
      fold_save->test_sizes.push_back(f_tests[k]->size());
    }
    rosbag::Bag bag;
    int bagmode = rosbag::bagmode::Write;
    bag.open("fold_save.bag", bagmode);
    bag.write("fold_save", ros::Time::now(), fold_save);
    bag.close();
    */
    
    MatrixXi confusion_mats[num_roc];
    for(int j=0;j<num_roc;j++) 
      confusion_mats[j] = MatrixXi::Zero(num_classes, num_classes);


    votes_total.resize(train_test_data->size());

    vector<float> roc_list;
    for(int i=0;i<NUM_FOLDS;i++) {
      RandomForest rf;
      printf("Growing Forest %d\n", i+1);
      //vector<int>* fold = new vector<int>();
      for(uint32_t k=0;k<folds[i]->size();k++)
        if((uint32_t) folds[i]->at(k) >= train_test_data->size())
          printf("WTF\n");
      //  cout << folds[i]->at(k) << " ";
      //rf.growForest(train_test_data, fold, c_num_trees);
      rf.growForest(train_test_data, folds[i], c_num_trees);
      //stringstream tmp; tmp << "forest_X_tmp_" << i << ".bag"; 
      //rf.writeForest(tmp.str());
      printf("Evaluating...\n");

      // maps traj_ids to a list of labels paired with vote maps
      map<int, pair<int, vector<pair<map<int, int>, float > >* > > votes_map;
      for(uint32_t j=0;j<f_tests[i]->size();j++) {
        int cur_traj_id = train_test_data->at(f_tests[i]->at(j))->traj_id;
        //vector<map<int, int> >* votes_list;
        if(votes_map.count(cur_traj_id) == 0) {
          pair<int, vector<pair<map<int, int>, float > >* > vote_pair;
          vote_pair.first = train_test_data->at(f_tests[i]->at(j))->label;
          vote_pair.second = new vector<pair<map<int, int>, float > >;
          votes_map[cur_traj_id] = vote_pair;
        }
        map<int, int> cur_votes;
        rf.collectVotes(train_test_data->at(f_tests[i]->at(j)), cur_votes);
        votes_map[cur_traj_id].second->push_back(
                make_pair(cur_votes, train_test_data->at(f_tests[i]->at(j))->detect_delay));
        for(map<int, int>::iterator vt_iter=cur_votes.begin();vt_iter!=cur_votes.end();
                     vt_iter++)
          votes_total[f_tests[i]->at(j)][vt_iter->first] = vt_iter->second;
        //cout << votes_list[j]->size() << " ";
      }
      float cur_percent = 1.0 / (num_roc + 1);

      for(int k=0;k<num_roc;k++) {
        map<int, pair<int, vector<pair<map<int, int>, float > >* > >::iterator votes_map_iter;
        for(votes_map_iter=votes_map.begin();
            votes_map_iter!=votes_map.end();votes_map_iter++) {
          int act_class = votes_map_iter->second.first;
          int pred_class;
          if(classify_first) 
            pred_class = findFirstClass(votes_map_iter->second.second, pos_id, 
                                        cur_percent * c_num_trees);
          else
            pred_class = findFrequentClass(votes_map_iter->second.second, pos_id, 
                                        cur_percent * c_num_trees);
          confusion_mats[k](label_cntr[pred_class],label_cntr[act_class])++;
        }
        if(i == 0)
          roc_list.push_back(cur_percent);
        cur_percent += 1.0 / (num_roc + 1);
        cout << confusion_mats[k] << endl;
      }
    }

    vector<vector<double> > tpr_list(num_roc);
    vector<vector<double> > fpr_list(num_roc);
    vector<vector<double> > spc_list(num_roc);
    for(int j=0;j<num_roc;j++) {
      for(int k=0;k<num_classes;k++) {
        double tpr = -1.0, fpr = -1.0, spc = -1.0;
        double TP = confusion_mats[j](k, k);
        double FP = confusion_mats[j].row(k).sum() - confusion_mats[j](k,k);
        double TN = confusion_mats[j].trace() - confusion_mats[j](k,k);
        double FN = confusion_mats[j].sum() - confusion_mats[j].row(k).sum() - TN;
        if((TP + FN) != 0)
          tpr = TP / (TP + FN);
        if((FP + TN) != 0)
          fpr = FP / (FP + TN);
        if(fpr != -1)
          spc = 1.0 - fpr;
        tpr_list[j].push_back(tpr);
        fpr_list[j].push_back(fpr);
        spc_list[j].push_back(spc);
      }
    }

    for(int k=0;k<num_classes;k++) {
      cout << "Class " << inv_label_cntr[k] << endl;
      cout << "ROC vals" << endl;
      for(int j=0;j<num_roc;j++)
        cout << roc_list[j] << ", ";
      cout << endl;
      cout << "TPR" << endl;
      for(int j=0;j<num_roc;j++)
        cout << tpr_list[j][k] << ", ";
      cout << endl;
      cout << "FPR" << endl;
      for(int j=0;j<num_roc;j++)
        cout << fpr_list[j][k] << ", ";
      cout << endl;
      cout << "SPC" << endl;
      for(int j=0;j<num_roc;j++)
        cout << spc_list[j][k] << ", ";
      cout << endl;
    }
  }

  void RandomForest::variableImportance() {
    loadDataset();
    loadForest();
    uint32_t num_feats = dataset->at(0)->features.size();
    int base_right = 0;
    int num_oobs = 0;
    vector<int> num_right(num_feats, 0);
    for(int i=0;i<num_trees;i++) {
      printf("Tree %d\n", i);
      int oobs_size = oobs[i].size();
      for(int j=0;j<oobs_size;j++) {
        int class_vote = trees[i]->classifyInstance(dataset->at(oobs[i][j]));
        base_right += class_vote == dataset->at(oobs[i][j])->label;
        num_oobs++;
      }
      uint32_t f;
      int j;
      int class_vote;
      for(f=0;f<num_feats;f++) {
        printf("Feature %d\n", f);
        vector<int> var_permute(oobs_size);
        vector<float> f_vals(oobs_size);
        for(j=0;j<oobs_size;j++) {
          var_permute[j] = oobs[i][j];
          f_vals[j] = dataset->at(var_permute[j])->features[f];
        }
        random_shuffle(var_permute.begin(), var_permute.end());
        int n_right = 0;
#pragma omp parallel default(shared) private(j, class_vote) num_threads(10) reduction( + : n_right)
    {
#pragma omp for schedule(dynamic, 10)
        for(j=0;j<oobs_size;j++) {
          dataset->at(oobs[i][j])->features[f] = f_vals[var_permute[j]];
          class_vote = trees[i]->classifyInstance(dataset->at(oobs[i][j]));
          n_right += class_vote == dataset->at(oobs[i][j])->label;
          dataset->at(oobs[i][j])->features[f] = f_vals[j];
        }
      }
        num_right[f] += n_right;
      }

    }
    for(uint32_t f=0;f<num_feats;f++) {
      float raw_score = (base_right - num_right[f]) / num_trees;
      printf("%4d score: %f\n", f, raw_score);
    }
  }

  void RandomForest::randomPermuteData() {
    loadDataset();

    rosbag::Bag bag;
    int bagmode;
    bagmode = rosbag::bagmode::Write;
    string bag_path, data_bag_name;
    nh_priv->getParam("data_bag_name", data_bag_name);
    nh_priv->getParam("bag_path", bag_path);
    string file_loc = bag_path + data_bag_name;
    bag.open(file_loc, bagmode);
    for(uint32_t i=0;i<dataset->size();i++) {
      SensorPoint sp;
      sp.features.resize(dataset->at(0)->features.size());
      for(uint32_t j=0;j<dataset->at(0)->features.size();j++)
        sp.features[j] = dataset->at(rand() % dataset->size())->features[j];
      bag.write("/collision_data", ros::Time::now(), sp);
    }
    bag.close();
  }


  /*void RandomForest::runTests(vector< SensorPoint::ConstPtr >* test_data, 
                              vector<int>* test_labels, int num_roc = 10) {
    MatrixXi class_votes = MatrixXi::Zero(test_data->size(), trees->at(0)->num_classes);
    for(uint32_t i=0;i<test_data->size();i++) {
      class_votes.row(i) = collectVotes(test_data->at(i));
    }
    if(num_roc > 0) {
      // binary case where we can vary threshold
      VectorXf tpr = VectorXf::Zero(num_roc);
      VectorXf fpr = VectorXf::Zero(num_roc);
      float cur_percent = 1.0 / (num_roc + 1);
      for(int32_t i=0;i<num_roc;i++) {
        MatrixXi confusion_mat = MatrixXi::Zero(2, 2);
        for(int32_t j=0;j<class_votes.rows();j++) {
          uint32_t pred_class = 0;
          if(class_votes(j,0) / (float) class_votes.row(j).sum() < cur_percent)
            pred_class = 1;
          uint32_t actual_class = test_labels[j];
          confusion_mat(actual_class, pred_class)++;
        }
        cout << "Percentage on class 0:" << cur_percent << endl;
        cout << confusion_mat << endl;
        cur_percent += 1.0 / (num_roc + 1);
      }
    } else {
      // multi-class case where we only consider popular vote
      MatrixXi confusion_mat = MatrixXi::Zero(trees->at(0)->num_classes, trees->at(0)->num_classes);
      for(int32_t i=0;i<class_votes.cols();i++) {
        int32_t pred_class = 0, pred_class_votes = 0;
        for(int32_t j=0;j<class_votes.rows();j++) {
          if(class_votes(j, i) > pred_class_votes) {
            pred_class = j;
            pred_class_votes = class_votes(j, i);
          }
        }
        uint32_t actual_class = test_data->at(i)->label;
        confusion_mat(actual_class, pred_class)++;
      }
      cout << confusion_mat << endl;
    }
  }*/
  double RandomForest::mahalanobisDist(LDLT<MatrixXd>* cov_inv, VectorXd& means, VectorXd& pt) {
    return sqrt( (pt - means).dot(cov_inv->solve(pt - means)) );
  }

  void RandomForest::doMahalanobis() {
    loadDataset();
    vector<vector< SensorPoint::Ptr > > datasets(1000); // split by label
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

    vector<LDLT<MatrixXd>* > cov_inv(num_classes);

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
    int k=0, j=0, l=0;
    uint32_t i=0;
    for(k=0;k<num_classes;k++) {
      //ROS_INFO("k %d, num_classes %d", j, num_classes);
      MatrixXd var_mat = MatrixXd::Zero(num_feats, num_feats);
#pragma omp parallel default(shared) private(j, l, i) num_threads(10)
    {
#pragma omp for schedule(dynamic, 10)
      for(j=0;j<num_feats;j++) {
        ROS_INFO("j %d", j);
        for(l=0;l<num_feats;l++) {
          for(i=0;i<datasets[k].size();i++) 
            var_mat(j,l) += (datasets[k][i]->features[j] - means[k](j)) * (datasets[k][i]->features[l] - means[k](l));
          var_mat(j,l) /= datasets[k].size();
        }
      }
    }
      //cout<< var_mat << endl;
      LDLT<MatrixXd>* qr = new LDLT<MatrixXd>(var_mat);
      cov_inv[k] = qr;
    }
    for(int k=0;k<num_classes;k++) {
      for(int l=0;l<num_classes;l++) {
        ArrayXd dists(datasets[l].size());
        VectorXd feat_vec(num_feats);
        int ind = 0, data_size = datasets[l].size();
#pragma omp parallel default(shared) private(ind, j) num_threads(10)
    {
#pragma omp for schedule(dynamic, 10)
        for(ind=0;ind<data_size;ind++) {
          for(j=0;j<num_feats;j++) 
            feat_vec(j) = datasets[l][ind]->features[j];
          dists[ind] = mahalanobisDist(cov_inv[k], means[k], feat_vec);
        }
    }
        int nans = 0;
        if(dists[i] != dists[i]) {
          dists[i] = 0;
          nans++;
        }
        //cout << dists << endl;
        double mean_dist = dists.sum() / (dists.size() - nans);
        VectorXd diff = dists - mean_dist;
        double var_dist = diff.dot(diff) / (dists.size() - nans);
        double std_dist = sqrt(var_dist);
        double min_dist = dists.minCoeff();
        double max_dist = dists.maxCoeff();
        printf("cov %d, data %d, mean_dist %f, std_dist %f, min %f, max %f, nans %d\n", k, l, mean_dist, std_dist, min_dist, max_dist, nans);
      }
      printf("cov %d, num_samps %d, rank 0\n", k, (int) datasets[k].size()); //, cov_inv[k]->rank());
    }
    printf("num_classes %d, num_feats %d\n", num_classes, num_feats);
  }

  void RandomForest::onInit() {
    nh = new ros::NodeHandle;
    nh_priv = new ros::NodeHandle("~");
    
    std::string results_topic, classify_topic, data_bag, forest_bag, loaded_topic;
    bool random_permute, training_mode, means, variable_import;

    nh_priv->param<bool>("is_abs", is_abs, false);

    nh_priv->param<bool>("variable_import", variable_import, false);
    if(variable_import) {
      variableImportance();
      return;
    }

    nh_priv->param<bool>("random_permute", random_permute, false);
    if(random_permute) {
      randomPermuteData();
      return;
    }

    nh_priv->param<bool>("mahalanobis", means, false);
    if(means) {
      doMahalanobis();
      return;
    }

    nh_priv->param<bool>("training_mode", training_mode, false);
    if(training_mode) {
      growWriteForest();
      return;
    }

    nh_priv->getParam("classify_topic", classify_topic);
    nh_priv->getParam("results_topic", results_topic);
    nh_priv->getParam("loaded_topic", loaded_topic);
    nh_priv->getParam("classifier_id", classifier_id);
    nh_priv->getParam("classifier_name", classifier_name);

    classify_sub = nh->subscribe(classify_topic.c_str(), 2, 
                        &RandomForest::classifyCallback, this);
    ROS_INFO("[random_forest] Subscribed to %s", classify_topic.c_str());
    results_pub = nh->advertise<ClassVotes>(results_topic, 1);
    ROS_INFO("[random_forest] Publishing on %s", results_topic.c_str());
    loaded_pub = nh->advertise<std_msgs::Bool>(loaded_topic, 1);
    ROS_INFO("[random_forest] Publishing on %s", loaded_topic.c_str());

    trees_loaded = false;
    loadForest();
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
  ros::init(argc, argv, "random_forest", ros::init_options::AnonymousName);
  //signal(SIGINT, INTHandler);

  RandomForest rf;
  rf.onInit();
  ros::spin();
  printf("Exiting\n");

  /*
  int num_attrs = 10;
  vector<SensorPoint::ConstPtr>* train = new vector<SensorPoint::ConstPtr>;
  for(int i=0;i<100;i++) {
    SensorPoint::Ptr add_data(new SensorPoint());
    add_data->features.resize(num_attrs);
    for(int j=0;j<num_attrs;j++) // add positives
      add_data->features[j] = 5.0 * rand() / double(RAND_MAX);
    add_data->traj_id = i / 10;
    add_data->label = 234;
    train->push_back(add_data);
  }
  for(int i=0;i<100;i++) {
    SensorPoint::Ptr add_data(new SensorPoint());
    add_data->features.resize(num_attrs);
    for(int j=0;j<num_attrs;j++) // add positives
      add_data->features[j] = 5.0 * rand() / double(RAND_MAX) + 5.1;
    add_data->traj_id = i / 10 + 10;
    add_data->label = 234;
    train->push_back(add_data);
  }
  for(int i=0;i<100;i++) {
    SensorPoint::Ptr add_data(new SensorPoint());
    add_data->features.resize(num_attrs);
    for(int j=0;j<num_attrs;j++) // add positives
      add_data->features[j] = 5.0 * rand() / double(RAND_MAX) + 70.0;
    add_data->traj_id = i / 10 + 20;
    add_data->label = 234;
    train->push_back(add_data);
  }
  for(int i=0;i<3000;i++) {
    SensorPoint::Ptr add_data(new SensorPoint());
    add_data->features.resize(num_attrs);
    for(int j=0;j<num_attrs;j++) // add negatives
      add_data->features[j] = 5.1 * (rand() / double(RAND_MAX));
    add_data->traj_id = i / 10 + 30;
    add_data->label = 51;
    train->push_back(add_data);
  }

  vector<SensorPoint::ConstPtr>* test = new vector<SensorPoint::ConstPtr>;
  for(int i=0;i<1000;i++) {
    SensorPoint::Ptr add_data(new SensorPoint());
    add_data->features.resize(num_attrs);
    for(int j=0;j<num_attrs;j++) // add positives
      add_data->features[j] = 5.0 * rand() / double(RAND_MAX);
    add_data->label = 1;
    test->push_back(add_data);
  }
  for(int i=0;i<1000;i++) {
    SensorPoint::Ptr add_data(new SensorPoint());
    add_data->features.resize(num_attrs);
    for(int j=0;j<num_attrs;j++) // add positives
      add_data->features[j] = 5.0 * rand() / double(RAND_MAX) + 20.0;
    add_data->label = 1;
    test->push_back(add_data);
  }
  for(int i=0;i<1000;i++) {
    SensorPoint::Ptr add_data(new SensorPoint());
    add_data->features.resize(num_attrs);
    for(int j=0;j<num_attrs;j++) // add positives
      add_data->features[j] = 5.0 * rand() / double(RAND_MAX) + 70.0;
    add_data->label = 1;
    test->push_back(add_data);
  }
  for(int i=0;i<3000;i++) {
    SensorPoint::Ptr add_data(new SensorPoint());
    add_data->features.resize(num_attrs);
    for(int j=0;j<num_attrs;j++) // add negatives
      add_data->features[j] = 10.0 * (rand() / double(RAND_MAX));
    add_data->label = 0;
    test->push_back(add_data);
  }

  if(0) {
    vector<int>* inds = new vector<int>(train->size());
    for(uint32_t i=0;i<train->size();i++) 
      inds->at(i) = i;
    RandomTree rt(0);
    rt.growTree(train, inds);
    for(int i=0;i<6000;i++)
      cout << rt.classifyInstance(test->at(i)) << ", " << train->at(i)->label << endl;
  }
  
  if(1) {
    vector<int>* inds = new vector<int>(train->size());
    for(uint32_t i=0;i<train->size();i++) 
      inds->at(i) = i;
    RandomForest rf;
    //rf.growForest(train, inds, 100);
    vector<int>* inds_test = new vector<int>(test->size());
    for(uint32_t i=0;i<test->size();i++) 
      inds_test->at(i) = i;
    RandomForest::runTenFold(train, 234, 20, 11);
    //rf.runTests(test, inds_test, 10);
  }*/


  return 0;
}
