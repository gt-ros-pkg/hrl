/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pr2_manipulation_controllers/JTTaskControllerState.h>

#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <control_toolbox/pid.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <rosrt/rosrt.h>

#include <filters/filter_chain.h>
#include <hrl_netft/HybridCartesianGains.h>

using namespace pr2_manipulation_controllers;

namespace hrl_netft {

template <int Joints>
struct Kin
{
  typedef Eigen::Matrix<double, Joints, 1> JointVec;
  typedef Eigen::Matrix<double, 6, Joints> Jacobian;

  Kin(const KDL::Chain &kdl_chain) :
    fk_solver_(kdl_chain), jac_solver_(kdl_chain),
    kdl_q(Joints), kdl_J(Joints)
  {
  }
  ~Kin()
  {
  }

  void fk(const JointVec &q, Eigen::Affine3d &x)
  {
    kdl_q.data = q;
    KDL::Frame kdl_x;
    fk_solver_.JntToCart(kdl_q, kdl_x);
    tf::transformKDLToEigen(kdl_x, x);
  }
  void jac(const JointVec &q, Jacobian &J)
  {
    kdl_q.data = q;
    jac_solver_.JntToJac(kdl_q, kdl_J);
    J = kdl_J.data;
  }

  KDL::ChainFkSolverPos_recursive fk_solver_;
  KDL::ChainJntToJacSolver jac_solver_;
  KDL::JntArray kdl_q;
  KDL::Jacobian kdl_J;
};

class HybridForceController : public pr2_controller_interface::Controller
{
public:
  // Ensure 128-bit alignment for Eigen
  // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
  enum { Joints = 7 };
  typedef Eigen::Matrix<double, Joints, 1> JointVec;
  typedef Eigen::Matrix<double, 6, 1> CartVec;
  typedef Eigen::Matrix<double, 6, Joints> Jacobian;
  typedef pr2_manipulation_controllers::JTTaskControllerState StateMsg;
public:
  HybridForceController();
  ~HybridForceController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  void starting();
  void update();

  Eigen::Affine3d x_desi_, x_desi_filtered_;
  CartVec wrench_desi_;

  ros::NodeHandle node_;
  ros::Subscriber sub_gains_;
  ros::Subscriber sub_posture_;
  ros::Subscriber sub_pose_;
  ros::Subscriber sub_force_; // khawkins
  ros::Subscriber sub_max_force_; // khawkins
  ros::Subscriber sub_ft_zero_; // khawkins
  ros::Subscriber sub_ft_params_; // khawkins
  tf::TransformListener tf_;

  rosrt::Publisher<StateMsg> pub_state_;
  rosrt::Publisher<geometry_msgs::PoseStamped> pub_x_, pub_x_desi_;
  rosrt::Publisher<geometry_msgs::Twist> pub_xd_, pub_xd_desi_;
  rosrt::Publisher<geometry_msgs::Twist> pub_x_err_, pub_wrench_;
  rosrt::Publisher<std_msgs::Float64MultiArray> pub_tau_;
  rosrt::Publisher<std_msgs::Float64MultiArray> pub_qdot_; // khawkins
  rosrt::Publisher<geometry_msgs::WrenchStamped> pub_sensor_ft_; // khawkins
  rosrt::Publisher<geometry_msgs::WrenchStamped> pub_sensor_raw_ft_; // khawkins
  rosrt::Publisher<geometry_msgs::WrenchStamped> pub_f_cmd_, pub_f_err_; // khawkins
  rosrt::Publisher<geometry_msgs::WrenchStamped> pub_k_effective_; // khawkins

  std::string root_name_, tip_name_;
  ros::Time last_time_;
  int loop_count_;
  pr2_mechanism_model::RobotState *robot_state_;

  pr2_mechanism_model::Chain chain_;
  boost::scoped_ptr<Kin<Joints> > kin_;
  Eigen::Matrix<double,6,1> Kp, Kd;  //aleeper
  Eigen::Matrix<double,6,6> St;  //aleeper
  bool use_tip_frame_; // aleeper
  Eigen::Matrix<double,6,1> Kfp, Kfi, Kfi_max, force_selector, motion_selector;  // khawkins
  double pose_command_filter_;
  double vel_saturation_trans_, vel_saturation_rot_;
  JointVec saturation_;
  JointVec joint_dd_ff_;
  double joint_vel_filter_;
  double jacobian_inverse_damping_;
  JointVec q_posture_;
  double k_posture_;
  bool use_posture_;

  // Minimum resolutions
  double res_force_, res_position_;
  double res_torque_, res_orientation_;

  Eigen::Affine3d last_pose_;
  CartVec last_wrench_;
  double last_stiffness_, last_compliance_;
  double last_Dx_, last_Df_;


  JointVec qdot_filtered_;

  // force/torque khawkins
  pr2_hardware_interface::AnalogIn *analog_in_;
  std::string force_torque_frame_;
  CartVec F_sensor_zero_;
  double gripper_mass_;
  Eigen::Vector3d gripper_com_;
  Eigen::Affine3d ft_transform_;
  CartVec F_des_, F_max_, F_integ_, K_effective_, F_err_last_;
  bool zero_wrench_;

  // filters khawkins
  std::vector<boost::shared_ptr<filters::FilterChain<double> > > force_filter_, qdot_filter_, xdot_filter_;

  void setGains(const hrl_netft::HybridCartesianGains::ConstPtr &msg) // khawkins
  {
    
    // Store gains...
    if (msg->p_gains.size() == 6)
      for (size_t i = 0; i < 6; ++i)
        Kp[i] = msg->p_gains[i];
    if (msg->d_gains.size() == 6)
      for (size_t i = 0; i < 6; ++i)
        Kd[i] = msg->d_gains[i];

    // Store force gains... khawkins
    if (msg->fp_gains.size() == 6)
      for (size_t i = 0; i < 6; ++i)
        Kfp[i] = msg->fp_gains[i];
    if (msg->fi_gains.size() == 6)
      for (size_t i = 0; i < 6; ++i)
        Kfi[i] = msg->fi_gains[i];
    if (msg->fi_max_gains.size() == 6)
      for (size_t i = 0; i < 6; ++i)
        Kfi_max[i] = msg->fi_max_gains[i];

    // Store selector matricies khawkins
    if (msg->force_selector.size() == 6)
        for (size_t i = 0; i < msg->force_selector.size(); ++i)
            if(msg->force_selector[i]) {
                force_selector[i] = 1;
                motion_selector[i] = 0;
            } else {
                force_selector[i] = 0;
                motion_selector[i] = 1;
            }

    // Store frame information
    if(!msg->header.frame_id.compare(root_name_))
    {
      use_tip_frame_ = false;
      ROS_INFO("New gains in root frame [%s]: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf",
               root_name_.c_str(), Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
      St.setIdentity();
    }
    else if(!msg->header.frame_id.compare(tip_name_))
    {
      use_tip_frame_ = true;
      ROS_INFO("New gains in tip frame [%s]: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf",
               tip_name_.c_str(), Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
      
    }
    else
    {
      use_tip_frame_ = false;
      
      geometry_msgs::PoseStamped in_root;
      in_root.pose.orientation.w = 1.0;
      in_root.header.frame_id = msg->header.frame_id;

      try {
        tf_.waitForTransform(root_name_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
        tf_.transformPose(root_name_, in_root, in_root);
      }
      catch (const tf::TransformException &ex)
      {
        ROS_ERROR("Failed to transform: %s", ex.what());
        return;
      }
      
      Eigen::Affine3d t;
      
      tf::poseMsgToEigen(in_root.pose, t);

      St << 
          t(0,0),t(0,1),t(0,2),0,0,0,
          t(1,0),t(1,1),t(1,2),0,0,0,
          t(2,0),t(2,1),t(2,2),0,0,0,
          0,0,0,t(0,0),t(0,1),t(0,2),
          0,0,0,t(1,0),t(1,1),t(1,2),
          0,0,0,t(2,0),t(2,1),t(2,2);
    
      St.transposeInPlace();
  
      ROS_INFO("New gains in arbitrary frame [%s]: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf",
             msg->header.frame_id.c_str(), Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
    }
  }

  void zeroFTSensor(const std_msgs::Bool::ConstPtr &msg) {
    zero_wrench_ = true;
    ROS_INFO("Sensor Zeroed");
  }

  void setFTSensorParams(const std_msgs::Float64MultiArray::ConstPtr &msg) // khawkins
  {
    if ((int)msg->data.size() == 1) {
      gripper_mass_ = msg->data[0];
    }
    else if ((int)msg->data.size() == 4) {
      gripper_mass_ = msg->data[0];
      gripper_com_[0] = msg->data[1];
      gripper_com_[1] = msg->data[2];
      gripper_com_[2] = msg->data[3];
    }
    else
    {
      ROS_ERROR("ft_params message had the wrong size: %d", (int)msg->data.size());
      return;
    }
  }

  void commandPosture(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    if (msg->data.size() == 0) {
      use_posture_ = false;
      ROS_INFO("Posture turned off");
    }
    else if ((int)msg->data.size() != q_posture_.size()) {
      ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
      return;
    }
    else
    {
      use_posture_ = true;
      for (int j = 0; j < Joints; ++j)
        q_posture_[j] = msg->data[j];
    }
  }

  void commandPose(const geometry_msgs::PoseStamped::ConstPtr &command)
  {
    geometry_msgs::PoseStamped in_root;
    try {
      tf_.waitForTransform(root_name_, command->header.frame_id, command->header.stamp, ros::Duration(0.1));
      tf_.transformPose(root_name_, *command, in_root);
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR("Failed to transform: %s", ex.what());
      return;
    }

    tf::poseMsgToEigen(in_root.pose, x_desi_);
  }

  void commandForce(const geometry_msgs::WrenchStamped::ConstPtr &msg)
  {
    F_des_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, 
              msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
    if(msg->header.frame_id == root_name_)
      return;

    geometry_msgs::PoseStamped in_root;
    in_root.pose.orientation.w = 1.0;
    in_root.header.frame_id = msg->header.frame_id;

    try {
      tf_.waitForTransform(root_name_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
      tf_.transformPose(root_name_, in_root, in_root);
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR("Failed to transform: %s", ex.what());
      return;
    }
    
    Eigen::Affine3d t;
    
    tf::poseMsgToEigen(in_root.pose, t);

    F_des_.head<3>() = t.linear() * F_des_.head<3>();
    F_des_.tail<3>() = t.linear() * F_des_.tail<3>();
  }

  void commandMaxForce(const geometry_msgs::WrenchStamped::ConstPtr &msg)
  {
    F_max_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, 
              msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  }

};


HybridForceController::HybridForceController()
  : robot_state_(NULL), use_posture_(false)
{}

HybridForceController::~HybridForceController()
{
  sub_gains_.shutdown();
  sub_posture_.shutdown();
  sub_pose_.shutdown();

  // khawkins
  sub_force_.shutdown();
  sub_max_force_.shutdown();
  sub_ft_zero_.shutdown();
  sub_ft_params_.shutdown();
}


bool HybridForceController::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &n)
{
  rosrt::init();
  node_ = n;

  ROS_INFO_STREAM("JTTask controller compiled at " << __TIME__ );
  // get name of root and tip from the parameter server
  // std::string tip_name; // aleeper: Should use the class member instead!
  if (!node_.getParam("root_name", root_name_)){
    ROS_ERROR("HybridForceController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tip_name", tip_name_)){
    ROS_ERROR("HybridForceController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // Chain of joints
  if (!chain_.init(robot_state_, root_name_, tip_name_))
    return false;
  if (!chain_.allCalibrated())
  {
    ROS_ERROR("Not all joints in the chain are calibrated (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }


  // Kinematics
  KDL::Chain kdl_chain;
  chain_.toKDL(kdl_chain);
  kin_.reset(new Kin<Joints>(kdl_chain));

  // Cartesian gains
  double kp_trans, kd_trans, kp_rot, kd_rot;
  if (!node_.getParam("cart_gains/trans/p", kp_trans) ||
      !node_.getParam("cart_gains/trans/d", kd_trans))
  {
    ROS_ERROR("P and D translational gains not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("cart_gains/rot/p", kp_rot) ||
      !node_.getParam("cart_gains/rot/d", kd_rot))
  {
    ROS_ERROR("P and D rotational gains not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  Kp << kp_trans, kp_trans, kp_trans,  kp_rot, kp_rot, kp_rot;
  Kd << kd_trans, kd_trans, kd_trans,  kd_rot, kd_rot, kd_rot;

  // aleeper
  use_tip_frame_ = false;
  if (!node_.getParam("use_tip_frame", use_tip_frame_)){
    ROS_WARN("HybridForceController: use_tip_frame was not specified, assuming 'false': %s)",
              node_.getNamespace().c_str());
  }
  St.setIdentity();

  // Force desired khawkins
  F_des_.setZero();
  F_integ_.setZero();
  K_effective_.setZero();

  double f_trans_max, f_rot_max;
  node_.param("force_trans_max", f_trans_max, 9999.0);
  node_.param("force_rot_max", f_rot_max, 9999.0);
  F_max_ << f_trans_max, f_trans_max, f_trans_max,  f_rot_max, f_rot_max, f_rot_max;

  // Force gains khawkins
  double kfp_trans, kfp_rot, kfi_trans, kfi_rot, kfi_max_trans, kfi_max_rot;
  if (!node_.getParam("force_gains/trans/p", kfp_trans) ||
      !node_.getParam("force_gains/trans/i", kfi_trans) ||
      !node_.getParam("force_gains/trans/i_max", kfi_max_trans))
  {
    ROS_ERROR("P and I translational force gains not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("force_gains/rot/p", kfp_rot) ||
      !node_.getParam("force_gains/rot/i", kfi_rot) ||
      !node_.getParam("force_gains/rot/i_max", kfi_max_rot))
  {
    ROS_ERROR("P and I rotational force gains not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  Kfp << kfp_trans, kfp_trans, kfp_trans,  kfp_rot, kfp_rot, kfp_rot;
  Kfi << kfi_trans, kfi_trans, kfi_trans,  kfi_rot, kfi_rot, kfi_rot;
  Kfi_max << kfi_max_trans, kfi_max_trans, kfi_max_trans,  kfi_max_rot, kfi_max_rot, kfi_max_rot;
  motion_selector = CartVec::Ones();
  force_selector = CartVec::Zero(); 
  force_filter_.resize(6);
  for(int i=0;i<6;i++) {
    force_filter_[i].reset(new filters::FilterChain<double>("double"));
    force_filter_[i]->configure("force_filter", node_);
    ros::Duration(0.2).sleep();
  }
  qdot_filter_.resize(Joints);
  for(int i=0;i<Joints;i++) {
    ros::Duration(0.2).sleep();
    qdot_filter_[i].reset(new filters::FilterChain<double>("double"));
    qdot_filter_[i]->configure("qdot_filter", node_);
  }
  xdot_filter_.resize(6);
  for(int i=0;i<6;i++) {
    ros::Duration(0.2).sleep();
    xdot_filter_[i].reset(new filters::FilterChain<double>("double"));
    xdot_filter_[i]->configure("xdot_filter", node_);
  }

  node_.param("pose_command_filter", pose_command_filter_, 1.0);

  // Velocity saturation
  node_.param("vel_saturation_trans", vel_saturation_trans_, 0.0);
  node_.param("vel_saturation_rot", vel_saturation_rot_, 0.0);

  node_.param("jacobian_inverse_damping", jacobian_inverse_damping_, 0.0);
  node_.param("joint_vel_filter", joint_vel_filter_, 1.0);

  // Joint gains
  for (int i = 0; i < Joints; ++i)
    node_.param("joint_feedforward/" + chain_.getJoint(i)->joint_->name, joint_dd_ff_[i], 0.0);
  for (int i = 0; i < Joints; ++i)
    node_.param("saturation/" + chain_.getJoint(i)->joint_->name, saturation_[i], 0.0);

  // Posture gains
  node_.param("k_posture", k_posture_, 1.0);

  node_.param("resolution/force", res_force_, 0.01);
  node_.param("resolution/position", res_position_, 0.001);
  node_.param("resolution/torque", res_torque_, 0.01);
  node_.param("resolution/orientation", res_orientation_, 0.001);

  // force/torque sensor khawkins
  zero_wrench_ = true;
  node_.param("gripper_params/mass", gripper_mass_, 1.02074);
  node_.param("gripper_params/com_pos_x", gripper_com_[0], -0.00126);
  node_.param("gripper_params/com_pos_y", gripper_com_[1], 0.001760);
  node_.param("gripper_params/com_pos_z", gripper_com_[2], -0.08532);
  std::string analog_in_name;
  if (!node_.getParam("force_torque_analog_in", analog_in_name))
  {
    ROS_ERROR("HybridForceController: No \"analog_in_name\" found on parameter namespace: %s",
              node_.getNamespace().c_str());
    return false;
  }
  pr2_hardware_interface::HardwareInterface* hw = robot_state_->model_->hw_;  
  analog_in_ = hw->getAnalogIn(analog_in_name);
  if (analog_in_ == NULL)
  {
    ROS_ERROR("HybridForceController: Cannot find AnalogIn named \"%s\"",
              analog_in_name.c_str());
    return false;
  }
  ROS_INFO("HybridForceController: Using AnalogIn named \"%s\"", analog_in_name.c_str());
  if (!node_.getParam("force_torque_frame", force_torque_frame_))
  {
    ROS_ERROR("HybridForceController: No \"force_torque_frame\" found on parameter namespace: %s",
              node_.getNamespace().c_str());
    return false;
  }
  try {
    tf::StampedTransform ft_stf;
    tf_.waitForTransform(tip_name_, force_torque_frame_, ros::Time(0), ros::Duration(1.0));
    tf_.lookupTransform(force_torque_frame_, tip_name_, ros::Time(0), ft_stf);
    tf::TransformTFToEigen(ft_stf, ft_transform_);
  }
  catch (const tf::TransformException &ex)
  {
    ROS_ERROR("Failed to transform: %s", ex.what());
    return false;
  }

  sub_gains_ = node_.subscribe("gains", 5, &HybridForceController::setGains, this);
  sub_posture_ = node_.subscribe("command_posture", 5, &HybridForceController::commandPosture, this);
  sub_pose_ = node_.subscribe("command_pose", 1, &HybridForceController::commandPose, this);
  // khawkins
  sub_force_ = node_.subscribe("command_force", 1, &HybridForceController::commandForce, this);
  sub_max_force_ = node_.subscribe("command_max_force", 1, &HybridForceController::commandMaxForce, this); 
  sub_ft_zero_ = node_.subscribe("ft_zero", 5, &HybridForceController::zeroFTSensor, this);
  sub_ft_params_ = node_.subscribe("ft_params", 5, &HybridForceController::setFTSensorParams, this);

  StateMsg state_template;
  state_template.header.frame_id = root_name_;
  state_template.x.header.frame_id = root_name_;
  state_template.x_desi.header.frame_id = root_name_;
  state_template.x_desi_filtered.header.frame_id = root_name_;
  state_template.tau_pose.resize(Joints);
  state_template.tau_posture.resize(Joints);
  state_template.tau.resize(Joints);
  state_template.J.layout.dim.resize(2);
  state_template.J.data.resize(6*Joints);
  state_template.N.layout.dim.resize(2);
  state_template.N.data.resize(Joints*Joints);
  pub_state_.initialize(node_.advertise<StateMsg>("state", 10), 10, state_template);

  geometry_msgs::PoseStamped pose_template;
  pose_template.header.frame_id = root_name_;
  pub_x_.initialize(node_.advertise<geometry_msgs::PoseStamped>("state/x", 10),
                    10, pose_template);
  pub_x_desi_.initialize(node_.advertise<geometry_msgs::PoseStamped>("state/x_desi", 10),
                         10, pose_template);
  pub_x_err_.initialize(node_.advertise<geometry_msgs::Twist>("x_err", 10),
                        10, geometry_msgs::Twist());
  pub_xd_.initialize(node_.advertise<geometry_msgs::Twist>("state/xd", 10),
                     10, geometry_msgs::Twist());
  pub_xd_desi_.initialize(node_.advertise<geometry_msgs::Twist>("state/xd_desi", 10),
                          10, geometry_msgs::Twist());
  pub_wrench_.initialize(node_.advertise<geometry_msgs::Twist>("state/wrench", 10),
                         10, geometry_msgs::Twist());
  pub_sensor_ft_.initialize(node_.advertise<geometry_msgs::WrenchStamped>("sensor_ft", 10),
                            10, geometry_msgs::WrenchStamped());
  pub_sensor_raw_ft_.initialize(node_.advertise<geometry_msgs::WrenchStamped>("sensor_raw_ft", 10),
                                10, geometry_msgs::WrenchStamped());
  pub_f_cmd_.initialize(node_.advertise<geometry_msgs::WrenchStamped>("f_cmd", 10),
                                10, geometry_msgs::WrenchStamped());
  pub_f_err_.initialize(node_.advertise<geometry_msgs::WrenchStamped>("f_err", 10),
                                10, geometry_msgs::WrenchStamped());
  pub_k_effective_.initialize(node_.advertise<geometry_msgs::WrenchStamped>("k_effective", 10),
                                10, geometry_msgs::WrenchStamped());

  std_msgs::Float64MultiArray joints_template;
  joints_template.layout.dim.resize(1);
  joints_template.layout.dim[0].size = Joints;
  joints_template.layout.dim[0].stride = 1;
  joints_template.data.resize(7);
  pub_tau_.initialize(node_.advertise<std_msgs::Float64MultiArray>("state/tau", 10),
                      10, joints_template);
  pub_qdot_.initialize(node_.advertise<std_msgs::Float64MultiArray>("qd", 10),
                       10, joints_template);

  return true;
}

void HybridForceController::starting()
{
  //Kp << 800.0, 800.0, 800.0,   80.0, 80.0, 80.0;
  //Kd << 12.0, 12.0, 12.0,   0.0, 0.0, 0.0;

  JointVec q;
  chain_.getPositions(q);
  kin_->fk(q, x_desi_);
  x_desi_filtered_ = x_desi_;
  last_pose_ = x_desi_;
  q_posture_ = q;
  qdot_filtered_.setZero();
  last_wrench_.setZero();
  F_err_last_.setZero();

  last_stiffness_ = 0;
  last_compliance_ = 0;
  last_Dx_ = 0;
  last_Df_ = 0;

  loop_count_ = 0;

  F_integ_.setZero();
  //zero_wrench_ = true; //khawkins
}


static void computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d &xdes, Eigen::Matrix<double,6,1> &err)
{
  err.head<3>() = xact.translation() - xdes.translation();
  err.tail<3>()   = 0.5 * (xdes.linear().col(0).cross(xact.linear().col(0)) +
                          xdes.linear().col(1).cross(xact.linear().col(1)) +
                          xdes.linear().col(2).cross(xact.linear().col(2)));
}

void HybridForceController::update()
{
  // get time
  ros::Time time = robot_state_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;
  ++loop_count_;

  // ======== Measures current arm state

  JointVec q;
  chain_.getPositions(q);

  Eigen::Affine3d x;
  kin_->fk(q, x);

  Jacobian J;
  kin_->jac(q, J);


  /*
  JointVec qdot_raw;
  chain_.getVelocities(qdot_raw);
  for (int i = 0; i < Joints; ++i)
    qdot_filtered_[i] += joint_vel_filter_ * (qdot_raw[i] - qdot_filtered_[i]);
  JointVec qdot = qdot_filtered_;
  */

  // filter qdot using Savitzky-Golay differentiation
  JointVec qdot;
  double in_qdot, out_qdot;
  for(int i=0;i<Joints;i++) {
    in_qdot = q[i];
    qdot_filter_[i]->update(in_qdot, out_qdot);
    qdot[i] = out_qdot;
  }
  qdot = qdot / dt.toSec();
  if(loop_count_ < 100)
    qdot.setZero();

  // low pass filter on xdot
  CartVec xdot = J * qdot;
  double in_xdot, out_xdot;
  for(int i=0;i<6;i++) {
    in_xdot = xdot[i];
    xdot_filter_[i]->update(in_xdot, out_xdot);
    xdot[i] = out_xdot;
  }

  // ======== Controls to the current pose setpoint

  {
    Eigen::Vector3d p0(x_desi_filtered_.translation());
    Eigen::Vector3d p1(x_desi_.translation());
    Eigen::Quaterniond q0(x_desi_filtered_.linear());
    Eigen::Quaterniond q1(x_desi_.linear());
    q0.normalize();
    q1.normalize();

    tf::Quaternion tf_q0(q0.x(), q0.y(), q0.z(), q0.w());
    tf::Quaternion tf_q1(q1.x(), q1.y(), q1.z(), q1.w());
    tf::Quaternion tf_q = tf_q0.slerp(tf_q1, pose_command_filter_);

    Eigen::Vector3d p = p0 + pose_command_filter_ * (p1 - p0);
    //Eigen::Quaterniond q = q0.slerp(pose_command_filter_, q1);
    Eigen::Quaterniond q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
    //x_desi_filtered_ = q * Eigen::Translation3d(p);
    x_desi_filtered_ = Eigen::Translation3d(p) * q;
  }
  CartVec x_err, x_err_ctrl_frame;
  //computePoseError(x, x_desi_, x_err);
  computePoseError(x, x_desi_filtered_, x_err);

  if(use_tip_frame_)
  { 
      St << 
          x(0,0),x(0,1),x(0,2),0,0,0,
          x(1,0),x(1,1),x(1,2),0,0,0,
          x(2,0),x(2,1),x(2,2),0,0,0,
          0,0,0,x(0,0),x(0,1),x(0,2),
          0,0,0,x(1,0),x(1,1),x(1,2),
          0,0,0,x(2,0),x(2,1),x(2,2);
      St.transposeInPlace();
  }

  // HERE WE CONVERT CALCULATIONS TO THE FRAME IN WHICH GAINS ARE SPECIFIED!
  x_err_ctrl_frame = St * x_err;
  CartVec xdot_desi = Kp.array() * x_err_ctrl_frame.array() * -1.0;  // aleeper

  // Caps the cartesian velocity
  if (vel_saturation_trans_ > 0.0)
  {
    if (fabs(xdot_desi.head<3>().norm()) > vel_saturation_trans_)
      xdot_desi.head<3>() *= (vel_saturation_trans_ / xdot_desi.head<3>().norm());
  }
  if (vel_saturation_rot_ > 0.0)
  {
    if (fabs(xdot_desi.tail<3>().norm()) > vel_saturation_rot_)
      xdot_desi.tail<3>() *= (vel_saturation_rot_ / xdot_desi.tail<3>().norm());
  }

  CartVec xdot_ctrl_frame = St * xdot;
  CartVec F_damp = Kd.array() * xdot_ctrl_frame.array();
  CartVec F_cmd = motion_selector.array() * xdot_desi.array() + 
                  force_selector.array() * (St * F_des_).array() - F_damp.array();
  // saturation force with F_max
  /*
     TODO What was I thinking? TODO
  double force_sat_scaling = 1.0, torque_sat_scaling = 1.0;
  for(int i=0;i<3;i++)
      if(F_max_[i] >= 0.0)
          force_sat_scaling = std::min(force_sat_scaling, fabs(F_max_[i] / F_cmd[i]));
  for(int i=3;i<6;i++)
      if(F_max_[i] >= 0.0)
          torque_sat_scaling = std::min(torque_sat_scaling, fabs(F_max_[i] / F_cmd[i]));
  F_cmd.head<3>() = force_sat_scaling * F_cmd.head<3>();
  F_cmd.tail<3>() = torque_sat_scaling * F_cmd.tail<3>();
  */

  // Cap force with F_max
  for(int i=0;i<6;i++) {
      if(F_max_[i] >= 0.0) {
          if(F_cmd[i] > 0)
              F_cmd[i] = std::min(F_max_[i], F_cmd[i]);
          else
              F_cmd[i] = -std::min(F_max_[i], -F_cmd[i]);
      }
  }

  CartVec F_motion = F_cmd;

  ////////////////////// Process force/torque sensor khawkins ////////////////////
  CartVec F_sensor_raw, F_sensor_zeroed, F_grav;
  {
    for(int i=0;i<6;i++)
        F_sensor_raw[i] = analog_in_->state_.state_[i];
    // ft_transform_ : ft_frame B tip_frame
    // x : base_frame B tip_frame
    Eigen::Vector4d z_grav(0, 0, -1, 0);
    z_grav = -1 * ft_transform_.matrix() * x.matrix().inverse() * z_grav;
    F_grav.head<3>() = gripper_mass_ * 9.81 * z_grav.head<3>();
    Eigen::Vector3d F_grav_vec(F_grav[0], F_grav[1], F_grav[2]);
    Eigen::Vector3d torque_vec = gripper_com_.cross(F_grav_vec);
    F_grav.tail<3>() = torque_vec;
    if(zero_wrench_) {
        F_sensor_zero_ = F_sensor_raw - F_grav;
        zero_wrench_ = false;
    }
    F_sensor_zeroed = (F_sensor_raw - F_grav - F_sensor_zero_);

    // put wrench in tip_frame
    Eigen::Vector3d F_sensor_zeroed_force(F_sensor_zeroed[0], F_sensor_zeroed[1], F_sensor_zeroed[2]);  
    Eigen::Vector3d F_sensor_zeroed_torque(F_sensor_zeroed[3], F_sensor_zeroed[4], F_sensor_zeroed[5]);  
    F_sensor_zeroed_force = ft_transform_.linear().transpose() * F_sensor_zeroed_force;
    F_sensor_zeroed_torque = ft_transform_.linear().transpose() * F_sensor_zeroed_torque;
    Eigen::Vector3d torque_offset = ft_transform_.translation().cross(F_sensor_zeroed_torque);
    F_sensor_zeroed.head<3>() = F_sensor_zeroed_force;
    F_sensor_zeroed.tail<3>() = F_sensor_zeroed_torque + torque_offset;

    if(!use_tip_frame_) {
        // put wrench in gains frame
        F_sensor_zeroed.head<3>() = x.linear() * F_sensor_zeroed.head<3>();
        F_sensor_zeroed.tail<3>() = x.linear() * F_sensor_zeroed.tail<3>();
        F_sensor_zeroed = St * F_sensor_zeroed;
    }

    // filter wrench signal
    double in_F, out_F;
    for(int i=0;i<6;i++) {
      in_F = F_sensor_zeroed[i];
      force_filter_[i]->update(in_F, out_F);
      F_sensor_zeroed[i] = out_F;
    }
    if(loop_count_ < 100)
      F_sensor_zeroed.setZero();
  }
  /////////////////////////////////////////////////////////////////////////////////

  // Impedance control

  CartVec proportional_selector, integral_selector;
  proportional_selector.setZero();
  integral_selector.setZero();
  for(int i=0;i<6;i++)
      if(Kfi[i] != 0 || Kfi_max[i] != 0)
          integral_selector[i] = 1;
      else
          proportional_selector[i] = 1;
  CartVec F_control, F_control_p, F_control_i, F_err;
  F_err = F_cmd - F_sensor_zeroed;
  // Propotional with feed-forward:
  F_control_p = F_cmd.array() + Kfp.array() * F_err.array();

  // Propotional-Integral:
  F_integ_ = integral_selector.array() * (F_integ_.array() + Kfi.array() * F_err.array());
  for(int i=0;i<6;i++)
      if(F_integ_[i] > Kfi_max[i])
          F_integ_[i] = Kfi_max[i];
      else if(F_integ_[i] < -Kfi_max[i])
          F_integ_[i] = -Kfi_max[i];
  // Marc's anti-windup trick: set to zero when sign flips
  double velocity = std::sqrt(xdot(0) * xdot(0) + xdot(1) * xdot(1) + xdot(2) * xdot(2));
  for(int i=0;i<6;i++)
      if(((F_err[i] > 0) != (F_err_last_[i] > 0)) && velocity > 0.01)
          F_integ_[i] = 0;
  F_err_last_ = F_err;
  //F_control = F_cmd.array() + F_integ_.array() + Kfp.array() * (F_cmd - F_sensor_zeroed).array();
  F_control_i = F_integ_.array() + Kfp.array() * (F_cmd - F_sensor_zeroed).array();

  F_control = proportional_selector.array() * F_control_p.array() + 
              integral_selector.array() * F_control_i.array();


  // HERE WE CONVERT BACK TO THE ROOT FRAME SINCE THE JACOBIAN IS IN ROOT FRAME.
  //JointVec tau_pose = J.transpose() * St.transpose() * F_motion;
  JointVec tau_pose = J.transpose() * St.transpose() * F_control; // khawkins

  // ======== J psuedo-inverse and Nullspace computation

  // Computes pseudo-inverse of J
  Eigen::Matrix<double,6,6> I6; I6.setIdentity();
  Eigen::Matrix<double,6,6> JJt = J * J.transpose();
  Eigen::Matrix<double,6,6> JJt_inv;
  JJt_inv = JJt.inverse();
  Eigen::Matrix<double,6,6> JJt_damped = J * J.transpose() + jacobian_inverse_damping_ * I6;
  Eigen::Matrix<double,6,6> JJt_inv_damped;
  JJt_inv_damped = JJt_damped.inverse();
  Eigen::Matrix<double,Joints,6> J_pinv = J.transpose() * JJt_inv_damped;

  // Computes the nullspace of J
  Eigen::Matrix<double,Joints,Joints> I;
  I.setIdentity();
  Eigen::Matrix<double,Joints,Joints> N = I - J_pinv * J;

  // ======== Posture control

  // Computes the desired joint torques for achieving the posture
  JointVec tau_posture;
  tau_posture.setZero();
  if (use_posture_)
  {
    JointVec posture_err = q_posture_ - q;
    for (size_t j = 0; j < Joints; ++j)
    {
      if (chain_.getJoint(j)->joint_->type == urdf::Joint::CONTINUOUS)
        posture_err[j] = angles::normalize_angle(posture_err[j]);
    }

    for (size_t j = 0; j < Joints; ++j) {
      if (fabs(q_posture_[j] - 9999) < 1e-5)
        posture_err[j] = 0.0;
    }

    JointVec qdd_posture = k_posture_ * posture_err;
    tau_posture = joint_dd_ff_.array() * (N * qdd_posture).array();
  }

  JointVec tau = tau_pose + tau_posture;

  // ======== Torque Saturation
  double sat_scaling = 1.0;
  for (int i = 0; i < Joints; ++i) {
    if (saturation_[i] > 0.0)
      sat_scaling = std::min(sat_scaling, fabs(saturation_[i] / tau[i]));
  }
  JointVec tau_sat = sat_scaling * tau;

  chain_.addEfforts(tau_sat);


  // ======== Environment stiffness

  CartVec df = F_motion - last_wrench_;
  CartVec dx;
  computePoseError(last_pose_, x, dx);

  // Just in the Z direction for now

  double Df, Dx;
  if (fabs(dx[2]) >= res_position_)
    Df = df[2] * res_position_ / fabs(dx[2]);
  else
    Df = (1. - fabs(dx[2])/res_position_) * last_Df_ + df[2];
  if (fabs(df[2]) >= res_force_)
    Dx = dx[2] * res_force_ / fabs(df[2]);
  else
    Dx = (1. - fabs(df[2])/res_force_) * last_Dx_ + dx[2];
  last_Df_ = Df;
  last_Dx_ = Dx;

  double stiffness, compliance;
  if (fabs(dx[2]) >= res_position_)
    stiffness = fabs(df[2]) / fabs(dx[2]);
  else
    stiffness = (1 - fabs(dx[2])/res_position_) * last_stiffness_ + fabs(df[2]) / res_position_;
  if (fabs(df[2]) >= res_force_)
    compliance = fabs(dx[2]) / fabs(df[2]);
  else
    compliance = (1 - fabs(df[2])/res_force_) * last_compliance_ + fabs(dx[2]) / res_force_;

  last_pose_ = x;
  last_wrench_ = F_motion;
  last_stiffness_ = stiffness;
  last_compliance_ = compliance;

  // khawkins
  for(int i=0;i<6;i++)
      if(fabs(x_err_ctrl_frame[i]) > 0.001)
          K_effective_[i] = fabs(F_sensor_zeroed[i] / x_err_ctrl_frame[i]);

  if (loop_count_ % 10 == 0)
  {
    geometry_msgs::PoseStamped::Ptr pose_msg;
    geometry_msgs::Twist::Ptr twist_msg;
    geometry_msgs::WrenchStamped::Ptr wrench_msg;
    std_msgs::Float64MultiArray::Ptr q_msg;

    if (pose_msg = pub_x_.allocate()) {  // X
      pose_msg->header.stamp = time;
      tf::poseEigenToMsg(x, pose_msg->pose);
      pub_x_.publish(pose_msg);
    }

    if (pose_msg = pub_x_desi_.allocate()) {  // X desi
      pose_msg->header.stamp = time;
      tf::poseEigenToMsg(x_desi_, pose_msg->pose);
      pub_x_desi_.publish(pose_msg);
    }

    if (twist_msg = pub_x_err_.allocate()) {  // X err
      tf::twistEigenToMsg(x_err, *twist_msg);
      pub_x_err_.publish(twist_msg);
    }

    if (twist_msg = pub_xd_.allocate()) {  // Xdot
      tf::twistEigenToMsg(xdot_ctrl_frame, *twist_msg);
      pub_xd_.publish(twist_msg);
    }

    if (twist_msg = pub_xd_desi_.allocate()) {  // Xdot desi
      tf::twistEigenToMsg(xdot_desi, *twist_msg);
      pub_xd_desi_.publish(twist_msg);
    }

    if (twist_msg = pub_wrench_.allocate()) {  // F
      tf::twistEigenToMsg(F_motion, *twist_msg);
      pub_wrench_.publish(twist_msg);
    }

    if (wrench_msg = pub_sensor_ft_.allocate()) {  // F ft
      wrench_msg->header.stamp = time;
      wrench_msg->header.frame_id = tip_name_;
      tf::wrenchEigenToMsg(F_sensor_zeroed, wrench_msg->wrench);
      pub_sensor_ft_.publish(wrench_msg);
    }

    if (wrench_msg = pub_sensor_raw_ft_.allocate()) {  // F ft raw
      wrench_msg->header.stamp = time;
      wrench_msg->header.frame_id = force_torque_frame_;
      tf::wrenchEigenToMsg(F_sensor_raw, wrench_msg->wrench);
      pub_sensor_raw_ft_.publish(wrench_msg);
    }

    if (wrench_msg = pub_f_cmd_.allocate()) {  // F_cmd
      wrench_msg->header.stamp = time;
      wrench_msg->header.frame_id = tip_name_;
      tf::wrenchEigenToMsg(F_cmd, wrench_msg->wrench);
      pub_f_cmd_.publish(wrench_msg);
    }

    if (wrench_msg = pub_f_err_.allocate()) {  // F_err
      wrench_msg->header.stamp = time;
      wrench_msg->header.frame_id = tip_name_;
      tf::wrenchEigenToMsg(F_err, wrench_msg->wrench);
      pub_f_err_.publish(wrench_msg);
    }

    if (wrench_msg = pub_k_effective_.allocate()) {  // K_effective
      wrench_msg->header.stamp = time;
      wrench_msg->header.frame_id = tip_name_;
      tf::wrenchEigenToMsg(K_effective_, wrench_msg->wrench);
      pub_k_effective_.publish(wrench_msg);
    }

    if (q_msg = pub_tau_.allocate()) {  // tau
      for (size_t i = 0; i < Joints; ++i)
        q_msg->data[i] = tau[i];
      pub_tau_.publish(q_msg);
    }

    if (q_msg = pub_qdot_.allocate()) {  // qdot
      for (size_t i = 0; i < Joints; ++i)
        q_msg->data[i] = qdot[i];
      pub_qdot_.publish(q_msg);
    }

    StateMsg::Ptr state_msg;
    if (state_msg = pub_state_.allocate()) {
      state_msg->header.stamp = time;
      state_msg->x.header.stamp = time;
      tf::poseEigenToMsg(x, state_msg->x.pose);
      state_msg->x_desi.header.stamp = time;
      tf::poseEigenToMsg(x_desi_, state_msg->x_desi.pose);
      state_msg->x_desi_filtered.header.stamp = time;
      tf::poseEigenToMsg(x_desi_filtered_, state_msg->x_desi_filtered.pose);
      tf::twistEigenToMsg(x_err, state_msg->x_err);
      tf::twistEigenToMsg(xdot, state_msg->xd);
      tf::twistEigenToMsg(xdot_desi, state_msg->xd_desi);
      tf::wrenchEigenToMsg(F_motion, state_msg->F);
      tf::matrixEigenToMsg(J, state_msg->J);
      tf::matrixEigenToMsg(N, state_msg->N);
      for (size_t j = 0; j < Joints; ++j) {
        state_msg->tau_pose[j] = tau_pose[j];
        state_msg->tau_posture[j] = tau_posture[j];
        state_msg->tau[j] = tau[j];
      }
      state_msg->stiffness = stiffness;
      state_msg->compliance = compliance;
      state_msg->Df = Df / res_position_;
      state_msg->Dx = Dx / res_force_;
      state_msg->df = df[2];
      state_msg->dx = dx[2];
      pub_state_.publish(state_msg);
    }
  }
  
}

} //namespace

PLUGINLIB_DECLARE_CLASS(hrl_netft, HybridForceController, hrl_netft::HybridForceController, pr2_controller_interface::Controller)

