#include "pr2_overhead_grasping/collision_monitor.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(collision_detection, collision_monitor, collision_detection::CollisionMonitor, nodelet::Nodelet)

using namespace std;
using namespace pr2_overhead_grasping;

namespace collision_detection {

  void CollisionMonitor::onInit() {
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_priv = getMTPrivateNodeHandle();
    
    std::string arm_str;
    nh_priv.getParam("arm", arm_str);
    char arm = arm_str[0];

    nh_priv.getParam("collision_id", collision_id);
    nh_priv.getParam("collision_thresh", collision_thresh);
    XmlRpc::XmlRpcValue cl_topics;
    nh_priv.getParam("classifier_topics", cl_topics);
    for(int i=0;i<cl_topics.size();i++)
      class_subs.push_back(nh.subscribe(cl_topics[i], 1, 
                          &CollisionMonitor::checkCollision, this));
    msg_buffers.resize(cl_topics.size());
    nh_priv.param<bool>("rf_on", rf_on, true);
    nh_priv.getParam("rf_thresh", rf_thresh);
    nh_priv.getParam("rf_buf_len", rf_buf_len);
    detect_buffer.resize(rf_buf_len);

    nh_priv.param<bool>("sigma_on", sigma_on, true);
    nh_priv.getParam("sigma_thresh", sigma_thresh);
    nh_priv.getParam("sigma_buf_len", sigma_buf_len);
    nh_priv.getParam("sigma_num", sigma_num);
    sigma_buffer.resize(sigma_buf_len);
    sigma_sub = nh.subscribe("/collision_detection/collision_sigma", 1, 
                          &CollisionMonitor::sigmaCallback, this);
    zero_sigma = nh.serviceClient<std_srvs::Empty>("/collision_detection/zero_sigma");

    nh_priv.param<bool>("error_on", error_on, true);
    if(error_on) {
      XmlRpc::XmlRpcValue xml_min_errors, xml_max_errors;
      nh_priv.getParam("min_errors", xml_min_errors);
      for(int i=0;i<xml_min_errors.size();i++)
        min_errors.push_back(static_cast<double>(xml_min_errors[i]));
      nh_priv.getParam("max_errors", xml_max_errors);
      for(int i=0;i<xml_max_errors.size();i++)
        max_errors.push_back(static_cast<double>(xml_max_errors[i]));
    }

    ind = 0;
    sigma_ind = 0;

    collision_detected = true;

    if(arm == 'r') {
      //sf_sub = nh.subscribe("r_arm_collision_detect", 1, 
      //                    &CollisionMonitor::checkCollision, this);
      detect_pub = nh.advertise<std_msgs::Bool>("r_arm_collision_detected", 1);
      NODELET_INFO("[collision_monitor] Publishing on r_arm_collision_detected");
      descript_pub = nh.advertise<pr2_overhead_grasping::CollisionDescription>("r_arm_collision_description", 1);
      NODELET_INFO("[collision_monitor] Publishing on r_arm_collision_description");
      start_srv = nh.advertiseService("r_start_detection", &CollisionMonitor::srvStartDetection, this);
      NODELET_INFO("[collision_monitor] Service advertised at r_start_detection");
      stop_srv = nh.advertiseService("r_stop_detection", &CollisionMonitor::srvStopDetection, this);
      NODELET_INFO("[collision_monitor] Service advertised at r_stop_detection");
      trig_srv = nh.advertiseService("r_trigger_collision", &CollisionMonitor::srvTriggerCollision, this);
      NODELET_INFO("[collision_monitor] Service advertised at r_trigger_collision");

      if(error_on) 
        error_sub = nh.subscribe("r_arm_controller/state", 1, 
                                 &CollisionMonitor::errorCallback, this);
    } else {
      // doesn't work yet.
      //sf_sub = nh.subscribe("l_arm_collision_detect", 1, 
      //                    &CollisionMonitor::checkCollision, this);
      detect_pub = nh.advertise<std_msgs::Bool>("l_arm_collision_detected", 1);
      NODELET_INFO("[collision_monitor] Publishing on l_arm_collision_detected");
      descript_pub = nh.advertise<pr2_overhead_grasping::CollisionDescription>("l_arm_collision_description", 1);
      NODELET_INFO("[collision_monitor] Publishing on l_arm_collision_description");
      start_srv = nh.advertiseService("l_start_detection", &CollisionMonitor::srvStartDetection, this);
      NODELET_INFO("[collision_monitor] Service advertised at l_start_detection");
      stop_srv = nh.advertiseService("l_stop_detection", &CollisionMonitor::srvStopDetection, this);
      NODELET_INFO("[collision_monitor] Service advertised at l_start_detection");
      trig_srv = nh.advertiseService("l_trigger_collision", &CollisionMonitor::srvTriggerCollision, this);
      NODELET_INFO("[collision_monitor] Service advertised at l_trigger_collision");

      if(error_on) 
        error_sub = nh.subscribe("l_arm_controller/state", 1, 
                                 &CollisionMonitor::errorCallback, this);
    }
  }

  void CollisionMonitor::checkCollision(pr2_overhead_grasping::ClassVotes::ConstPtr class_votes) {
    // save message to buffer
    msg_buffers[class_votes->classifier_id] = class_votes;
    if(collision_detected || class_votes->classifier_id != 0) 
      // we stop running this callback until the monitor is restarted
      return;

    bool collision_positive = false;

    /////////////////////// RF ///////////////////////////
    int coll_ind = collision_id;
    int coll_votes = class_votes->votes[coll_ind];
    //if(coll_votes > collision_thresh)
      //collision_positive = true;
    detect_buffer[ind % detect_buffer.size()] = coll_votes;
    if(accumulate(detect_buffer.begin(), detect_buffer.end(), 0) >= rf_thresh)
      if(rf_on)
        collision_positive = true;

    // check for collision
    if(collision_positive) {
      triggerCollision();

      //start description thread
      descript_thread = boost::thread(&CollisionMonitor::describeCollision, this);
    }
    ind++;
  }

  void CollisionMonitor::startDetection() {
    start_time = ros::Time::now().toSec();
    collision_detected = false;
  }

  bool CollisionMonitor::srvStartDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    startDetection();
    return true;
  }

  bool CollisionMonitor::srvStopDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    collision_detected = true;
    return true;
  }

  void CollisionMonitor::triggerCollision() {
    if(!collision_detected) {
      collision_detected = true;
      std_msgs::Bool bool_true;
      bool_true.data = true;
      detect_pub.publish(bool_true);
    }
  }

  bool CollisionMonitor::srvTriggerCollision(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    triggerCollision();
    return true;
  }

  void CollisionMonitor::describeCollision() {
    // put description classifiers here

    int rate = 100;
    ros::Rate r(rate);
    vector<vector<int> > vote_counts(msg_buffers.size());
    vector<int> vote_sums(msg_buffers.size());
    for(uint32_t i=0;i<msg_buffers.size();i++) 
      vote_counts[i].resize(msg_buffers[i]->classes.size(), 0);
    int num_counts = 0;
    int descript_time = 5;

    while(ros::ok()) {
      ros::spinOnce();
      for(uint32_t i=0;i<msg_buffers.size();i++) {
        for(uint32_t j=0;j<msg_buffers[i]->classes.size();j++) {
          vote_counts[i][msg_buffers[i]->classes[j]] += msg_buffers[i]->votes[j];
          vote_sums[i] += msg_buffers[i]->votes[j];
        }
      }
      num_counts++;
      if(num_counts == descript_time)
        break;
      r.sleep();
    }

    CollisionDescription coll_descript;
    for(uint32_t i=0;i<vote_counts.size();i++) {
      int prediction = max_element(vote_counts[i].begin(), vote_counts[i].end()) - 
                              vote_counts[i].begin();
      coll_descript.classifier_names.push_back(msg_buffers[i]->classifier_name);
      coll_descript.predictions.push_back(prediction);
      coll_descript.p_votes.push_back(vote_counts[i][prediction] / (float) vote_sums[i] );
      coll_descript.sizes.push_back(vote_counts[i].size());
      for(uint32_t j=0;j<vote_counts[i].size();j++) 
        coll_descript.t_votes.push_back(vote_counts[i][j] / (float) vote_sums[i]);
    }
    descript_pub.publish(coll_descript);
  }

  void CollisionMonitor::sigmaCallback(std_msgs::Float64::ConstPtr msg) {
    if(ros::Time::now().toSec() - start_time < 1.2) {
      std_srvs::Empty emp;
      zero_sigma.call(emp);
      return;
    }
    sigma_buffer[sigma_ind % sigma_buf_len] = abs(msg->data) > sigma_thresh;

    /////////////////////// SIGMA ///////////////////////////
    if(accumulate(sigma_buffer.begin(), sigma_buffer.end(), 0) >= sigma_num)
      if(sigma_on)
        triggerCollision();

    sigma_ind++;
  }

  void CollisionMonitor::errorCallback(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr message) {
    for(int i=0;i<7;i++)
      if(message->error.positions[i] < min_errors[i] ||
         message->error.positions[i] > max_errors[i])
        if(error_on)
          triggerCollision();
  }

  CollisionMonitor::~CollisionMonitor() {
    descript_thread.interrupt();
  }

}
