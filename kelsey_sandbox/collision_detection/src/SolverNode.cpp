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

#include "collision_detection/SolverNode.h"
 
void SolverNode::lookupJointIDs(const std::vector<std::string>& names) {
  uint j, i;
  bool found = false;
  for (j = 0; j < joint_names.size(); j++) {
    found = false;
    for (i = 0; i < names.size(); i++)
      if (names[i] == joint_names[j]) {
        joint_ids.push_back(i);
        found = true;
        break;
      }
    if (!found)
      break;
  }
  
  if (!found) {
    ROS_ERROR("did not find joint %s in joint_state msg.", joint_names[j].c_str());
  }
}

void SolverNode::js_callback(const sensor_msgs::JointState& msg)
{
  if (0 == joint_ids.size())
    lookupJointIDs(msg.name);

  std::vector<double> pos(ndof);
  std::vector<double> vel(ndof);
  std::vector<double> acc(ndof);
  std::vector<double> effort(ndof);
  std::vector<double> computed_effort(ndof);

  double dt = (msg.header.stamp - last_time).toSec();
  
  for (uint i = 0; i < ndof; i++) {
    pos[i] = msg.position[joint_ids[i]]; // pos_filter[i]->getNextFilteredValue(msg.position[joint_ids[i]]);
    vel[i] = (pos[i] - last_pos[i]) / dt;
    //vel[i] = msg.velocity[joint_ids[i]];

    // acc[i] = (pos[i] - 2.0 * last_pos[i] + sec_last_pos[i]) / (dt * dt); // pos_filter[i]->getNextFilteredValue((vel[i] - last_vel[i]) / dt);
    computed_effort[i] = (vel[i] - last_vel[i]) / dt;
    acc[i] = acc_filter[i]->getNextFilteredValue((vel[i] - last_vel[i]) / dt ); // pos_filter[i]->getNextFilteredValue((vel[i] - last_vel[i]) / dt);


    last_pos[i] = pos[i];
    last_vel[i] = vel[i];

    // acc[i] = (double) acc_filter->
    //   getNextFilteredValue((float) msg.velocity[(vel[i] - last_vel[i]) / dt^2]);

    // vel[i] = (double) vel_filter->
    //   getNextFilteredValue((float) msg.velocity[joint_ids[i]]);
    // acc[i] = (double) acc_filter->
    //   getNextFilteredValue((float) msg.velocity[(vel[i] - last_vel[i]) / dt^2]);


    effort[i] = eff_filter[i]->getNextFilteredValue(msg.effort[joint_ids[i]]);
    

  }
  last_time = msg.header.stamp;

  // last_pos = std::vector<double>(ndof);
  // std::copy(pos.begin(), pos.end(), last_pos.begin()); 

  // last_vel = std::vector<double>(ndof);
  // std::copy(vel.begin(), vel.end(), last_vel.begin()); 

  solver->updateState(pos, vel, acc);
  solver->getTorques(computed_effort);

  std_msgs::Float64 sigma_msg;
  sigma_msg.data = solver->sigma;
  sigma_pub.publish(sigma_msg);
}

SolverNode::SolverNode(InverseDynamicsSolverWBC& id_solver, 
                             std::vector<std::string>& joints) {
  solver = &id_solver;
  joint_names = joints;
  ndof = joint_names.size();
  if (ndof != solver->getNumDOF()) {
    ROS_ERROR("expected solver with %zu DOF but got %zu", ndof, solver->getNumDOF());
    exit(-1);
  }

  last_time = ros::Time::now();
  last_pos.resize(ndof);
  sec_last_pos.resize(ndof);
  last_vel.resize(ndof);

  // double filt_pos_b[] = {0.0009,    0.0019,    0.0009};
  // double filt_pos_a[] = {1.0000,   -1.8782,    0.8819};
  double filt_pos_b[] = {0.0035,    0.0070,    0.0035}; // this one is pretty good
  double filt_pos_a[] = {1.0000,   -1.7635,    0.7775};
  
  pos_filter.resize(ndof);
  for (uint i = 0; i < pos_filter.size(); i++) {
    pos_filter[i] = new digitalFilter(2, true, &filt_pos_b[0], &filt_pos_a[0]);
  }

  double filt_vel_b[] = {0.0035,    0.0070,    0.0035}; // this one is pretty good
  double filt_vel_a[] = {1.0000,   -1.7635,    0.7775};
  vel_filter.resize(ndof);
  for (uint i = 0; i < pos_filter.size(); i++) {
    vel_filter[i] = new digitalFilter(2, true, &filt_vel_b[0], &filt_vel_a[0]);
  }


  double filt_acc_b[] = {0.0035,    0.0070,    0.0035}; // this one is pretty good
  double filt_acc_a[] = {1.0000,   -1.7635,    0.7775};
  acc_filter.resize(ndof);
  for (uint i = 0; i < acc_filter.size(); i++) {
    acc_filter[i] = new digitalFilter(2, true, filt_acc_b, filt_acc_a);
  }

  double filt_eff_b[] = {0.0035,    0.0070,    0.0035}; // this one is pretty good
  double filt_eff_a[] = {1.0000,   -1.7635,    0.7775};
  eff_filter.resize(ndof);
  for (uint i = 0; i < eff_filter.size(); i++) {
    eff_filter[i] = new digitalFilter(2, true, filt_eff_b, filt_eff_a);
  }



  ros::NodeHandle n;
  joint_state_pub = 
    n.advertise<collision_detection::JointState>("dynamics_joint_states", 1);

  joint_state_sub = 
    n.subscribe("/joint_states", 20, &SolverNode::js_callback, this);  

  sigma_pub = 
    n.advertise<std_msgs::Float64>("collision_sigma", 1);
}
