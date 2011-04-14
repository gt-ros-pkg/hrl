/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/*
  Author: Daniel Hennes
 */

#include "inverse_dynamics/id_controller.h"
#include <pluginlib/class_list_macros.h>

namespace controller {

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(inverse_dynamics,
                        IDController, 
                        controller::IDController, 
                        pr2_controller_interface::Controller)


/// Controller initialization in non-realtime
bool IDController::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n)
{
  robot_ = robot;
  node_ = n;

  // solver
  std::string sai_xml_fname;
  if (!node_.getParam("sai_xml", sai_xml_fname))
  {
    ROS_ERROR("No sai_xml filename given. (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  ROS_INFO("sai_xml filename: %s", sai_xml_fname.c_str());
  solver_ = new InverseDynamicsSolverWBC(sai_xml_fname);

  // get all joints
  XmlRpc::XmlRpcValue joint_names;
  if (!node_.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpc::XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                node_.getNamespace().c_str());
      return false;
    }

    pr2_mechanism_model::JointState *j = robot->getJointState((std::string)name_value);
    if (!j) {
      ROS_ERROR("Joint not found: %s. (namespace: %s)",
                ((std::string)name_value).c_str(), node_.getNamespace().c_str());
      return false;
    }
    joints_.push_back(j);
  }

  // filters
  std::string p = "acc_filter";
  if (node_.hasParam(p)) {
    acc_filter_ = new filters::MultiChannelFilterChain<double>("double");
    if (!acc_filter_->configure(joints_.size(), node_.resolveName(p).c_str())) {
      ROS_ERROR("Filters not configured, %s. (namespace %s)", p.c_str(),
                node_.getNamespace().c_str());
      return false;
    }
  }
  p = "vel_filter";
  if (node_.hasParam(p)) {
    vel_filter_ = new filters::MultiChannelFilterChain<double>("double");
    if (!vel_filter_->configure(joints_.size(), node_.resolveName(p).c_str())) {
      ROS_ERROR("Filters not configured, %s. (namespace %s)", p.c_str(),
                node_.getNamespace().c_str());
      return false;
    }
  }
  p = "pos_filter";
  if (node_.hasParam(p)) {
    pos_filter_ = new filters::MultiChannelFilterChain<double>("double");
    if (!pos_filter_->configure(joints_.size(), node_.resolveName(p).c_str())) {
      ROS_ERROR("Filters not configured, %s. (namespace %s)", p.c_str(),
                node_.getNamespace().c_str());
      return false;
    }
  }
  p = "eff_filter";
  if (node_.hasParam(p)) {
    eff_filter_ = new filters::MultiChannelFilterChain<double>("double");
    if (!eff_filter_->configure(joints_.size(), node_.resolveName(p).c_str())) {
      ROS_ERROR("Filters not configured, %s. (namespace %s)", p.c_str(),
                node_.getNamespace().c_str());
      return false;
    }
  }


  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<inverse_dynamics::JointState> (node_, "joint_states", 1));

  controller_state_publisher_->lock();
  for (size_t j = 0; j < joints_.size(); ++j)
    controller_state_publisher_->msg_.name.push_back(joints_[j]->joint_->name);
  controller_state_publisher_->msg_.position.resize(joints_.size());
  controller_state_publisher_->msg_.velocity.resize(joints_.size());
  controller_state_publisher_->msg_.acceleration.resize(joints_.size());
  controller_state_publisher_->msg_.effort.resize(joints_.size());
  controller_state_publisher_->msg_.computed_effort.resize(joints_.size());
  controller_state_publisher_->unlock();

  return true;
}


/// Controller startup in realtime
void IDController::starting() {
  last_vel_.resize(joints_.size());
  last_time_ = robot_->getTime();
}


/// Controller update loop in realtime
void IDController::update() {
  ros::Time time = robot_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  std::vector<double> pos(joints_.size());
  std::vector<double> vel(joints_.size());
  std::vector<double> acc(joints_.size());
  std::vector<double> eff(joints_.size());
  std::vector<double> computed_eff(joints_.size());


  for (size_t j = 0; j < joints_.size(); j++) {
    pos[j] = joints_[j]->position_;
    vel[j] = joints_[j]->velocity_;
    acc[j] = (vel[j] - last_vel_[j]) / dt.toSec();
    eff[j] = joints_[j]->measured_effort_;
  }

  // vel_filter_->update(vel, vel);
  acc_filter_->update(acc, acc);
  eff_filter_->update(eff, eff);

  solver_->updateState(pos, vel, acc);
  solver_->getTorques(computed_eff);

  if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
    controller_state_publisher_->msg_.header.stamp = time;
    controller_state_publisher_->msg_.position = pos;
    controller_state_publisher_->msg_.velocity = vel;      
    controller_state_publisher_->msg_.acceleration = acc;
    controller_state_publisher_->msg_.effort = eff;
    controller_state_publisher_->msg_.computed_effort = computed_eff;
    controller_state_publisher_->unlockAndPublish();
  }

  std::copy(vel.begin(), vel.end(), last_vel_.begin());

}

/// Controller stopping in realtime
void IDController::stopping() { 

}

} // namespace
