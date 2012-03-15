#include "pr2_overhead_grasping/force_torque_monitor.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(collision_detection, force_torque_monitor, collision_detection::ForceTorqueMonitor, nodelet::Nodelet)

namespace collision_detection {

  void ForceTorqueMonitor::onInit() {
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_priv = getMTPrivateNodeHandle();

    std::string arm_str;
    nh_priv.getParam("arm", arm_str);
    char arm = arm_str[0];
    nh_priv.getParam("z_thresh", z_thresh);
    nh_priv.getParam("delay_time", delay_time);
    //nh_priv.getParam("label", label);

    collision_detected = true;
    collision_time = ros::Time::now().toSec() - 1000.0;

    ft_sub = nh.subscribe("force_torque_ft1_Vec3", 1, 
                        &ForceTorqueMonitor::checkCollision, this);
    detect_pub = nh.advertise<std_msgs::Bool>("force_torque_collision", 1);
    NODELET_INFO("[force_torque_monitor] Publishing on force_torque_collision");
    start_srv = nh.advertiseService("ft_start_detection", &ForceTorqueMonitor::srvStartDetection, this);
    NODELET_INFO("[force_torque_monitor] Service advertised at ft_start_detection");
    stop_srv = nh.advertiseService("ft_stop_detection", &ForceTorqueMonitor::srvStopDetection, this);
    NODELET_INFO("[force_torque_monitor] Service advertised at ft_stop_detection");
    coll_pub = nh.advertise<pr2_overhead_grasping::SensorPoint>("collision_data", 20);
    NODELET_INFO("[force_torque_monitor] Publishing on collision_data");

    if(arm == 'r') {
      sf_sub = nh.subscribe("r_arm_features", 1, 
                          &ForceTorqueMonitor::saveCollision, this);
    } else {
      sf_sub = nh.subscribe("l_arm_features", 1, 
                          &ForceTorqueMonitor::saveCollision, this);
    }
    traj_ind = 0;
  }

  void ForceTorqueMonitor::checkCollision(geometry_msgs::Vector3Stamped::ConstPtr force) {
    if(!is_collision)
      return;
    if(collision_detected) 
      // we stop running this callback until the monitor is restarted
      return;
    if(force->vector.z >= z_thresh) {
      collision_time = ros::Time::now().toSec();
      collision_detected = true;
      ros::NodeHandle nh = getMTNodeHandle();
      delay_timer = nh.createTimer(ros::Duration(delay_time), &ForceTorqueMonitor::endCollision, this);
    }
  }

  void ForceTorqueMonitor::endCollision(const ros::TimerEvent& event) {
    std_msgs::Bool bool_true;
    bool_true.data = true;
    detect_pub.publish(bool_true);
    delay_timer.stop();
  }

  void ForceTorqueMonitor::saveCollision(pr2_overhead_grasping::SensorPoint::ConstPtr sp) {
    double cur_delay = ros::Time::now().toSec() - collision_time;
    if(is_collision) {
      if(!collision_detected || cur_delay > delay_time) 
        // we stop running this callback until the monitor is restarted
        return;
    } else {
      if(collision_detected) 
        return;
    }

    pr2_overhead_grasping::SensorPoint new_sp(*sp);
    new_sp.detect_delay = cur_delay;
    new_sp.label = label;
    new_sp.traj_id = label * 1000 + traj_ind;
    coll_pub.publish(new_sp);
  }

  void ForceTorqueMonitor::startDetection() {
    collision_detected = false;
  }

  void ForceTorqueMonitor::stopDetection() {
    collision_time = ros::Time::now().toSec() - 1000.0;
    collision_detected = true;
    traj_ind++;
  }

  bool ForceTorqueMonitor::srvStartDetection(pr2_overhead_grasping::StartFTDetect::Request& req, pr2_overhead_grasping::StartFTDetect::Response& resp) {
    label = req.label;
    is_collision = req.is_collision;
    startDetection();
    return true;
  }

  bool ForceTorqueMonitor::srvStopDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    stopDetection();
    return true;
  }

}
