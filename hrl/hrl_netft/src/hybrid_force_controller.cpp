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

#include "hrl_netft/hybrid_force_controller.h"
#include "pluginlib/class_list_macros.h"
#include <math.h>
#include <boost/foreach.hpp>
#include <string>
#include <algorithm>

PLUGINLIB_DECLARE_CLASS(hrl_netft, HybridForceController, hrl_netft::HybridForceController, pr2_controller_interface::Controller)


namespace hrl_netft
{

HybridForceController::HybridForceController() : 
    max_force_(0.0),
    max_torque_(0.0),
    analog_in_(NULL),
    pub_cycle_count_(0),
    should_publish_(false),
    base_link("torso_lift_link"),
    netft_link("l_netft_frame"),
    tool_link("l_gripper_tool_frame")
{}

HybridForceController::~HybridForceController() {}

bool HybridForceController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node)
{
    if (!robot) 
        return false;

    ////////////////////////////// NetFT Setup /////////////////////////
    std::string analog_in_name;
    if (!node.getParam("analog_in_name", analog_in_name)) {
        ROS_ERROR("HybridForceController: No \"analog_in_name\" found on parameter namespace: %s",
                node.getNamespace().c_str());
        return false;
    }

    pr2_hardware_interface::HardwareInterface* hw = robot->model_->hw_;  
    analog_in_ = hw->getAnalogIn(analog_in_name);
    if (analog_in_ == NULL) {
        ROS_ERROR("HybridForceController: Cannot find AnalogIn named \"%s\"",
                analog_in_name.c_str());
        BOOST_FOREACH(const pr2_hardware_interface::AnalogInMap::value_type &v, hw->analog_ins_) {
            ROS_INFO("AnalogIn : %s", v.first.c_str());
        }
        return false;
    }
    ROS_INFO("HybridForceController: Using AnalogIn named \"%s\"", analog_in_name.c_str());
    ////////////////////////////////////////////////////////////////////

    ///////////////////////// KDL Setup ////////////////////////////////
    if(!netft_chain_.init(robot, base_link.c_str(), netft_link.c_str()))
        return false;
    netft_chain_.toKDL(netft_kdl_chain_);
    netft_kin_.reset(new Kin(netft_kdl_chain_));
    if(!tool_chain_.init(robot, base_link.c_str(), tool_link.c_str()))
        return false;
    tool_chain_.toKDL(tool_kdl_chain_);
    tool_kin_.reset(new Kin(tool_kdl_chain_));

    // Initialize realtime publisher to publish to ROS topic 
    pub_state_.init(node, "state", 2);
    ////////////////////////////////////////////////////////////////////

    return true;
}


void HybridForceController::starting() {
}


void HybridForceController::update() {
    if(analog_in_->state_.state_.size() != 6) {
        ROS_ERROR_THROTTLE(5.0, "HybridForceController: AnalogInput is has unexpected size %d", 
                int(analog_in_->state_.state_.size()));
        return;
    }

    for(int i=0;i<6;i++)
        ft_wrench_(i) = analog_in_->state_.state_[i];

    // Publish data in ROS message every 10 cycles (about 100Hz)
    if(++pub_cycle_count_ > 10) {
        should_publish_ = true;
        pub_cycle_count_ = 0;
    }

    if(should_publish_ && pub_state_.trylock()) {
        should_publish_ = false;
        eigenToWrench(ft_wrench_, pub_state_.msg_.tool_force_sensed.wrench);
        pub_state_.unlockAndPublish();
    }
}

void eigenToWrench(const CartVec& eig_w, geometry_msgs::Wrench& ros_w) {
    ros_w.force.x = eig_w(0); ros_w.force.y = eig_w(1); ros_w.force.z = eig_w(2); 
    ros_w.torque.x = eig_w(3); ros_w.torque.y = eig_w(4); ros_w.torque.z = eig_w(5); 
}

}//namespace hrl_netft
