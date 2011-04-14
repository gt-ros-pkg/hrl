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

#include "collision_detection/InverseDynamicsSolverWBC.h"

InverseDynamicsSolverWBC::InverseDynamicsSolverWBC(std::string fname) {
  // load sai model
  try {
    jspace::test::BRParser brp;
    jspace::test::BranchingRepresentation * brep(brp.parse(fname));
    jspace::tao_tree_info_s * tree(brep->createTreeInfo());
    if (0 != model.init(tree, 0, &std::cout)) {
      ROS_ERROR("failed to load jspace model from %s!", fname.c_str());
      exit(-1);
    }
  } catch (std::exception const & ee) {
    ROS_ERROR("failed to parse sai model from %s! (%s)", fname.c_str(), ee.what());
    exit(-1);
  }
  ndof = model.getNDOF(); // set DOF
  last_time = ros::Time::now().toSec();
  r = jspace::Vector::Zero(7);
  nh_priv = new ros::NodeHandle("~");
  js_sub = nh.subscribe("/joint_states", 1, 
                        &InverseDynamicsSolverWBC::jsCallback, this);
  ros::Rate r(100);
  first_call = false;
  while(ros::ok()) {
    // Call callbacks to get most recent data
    ros::spinOnce();
    if(first_call)
      break;
    r.sleep();
  }
  pub = nh.advertise<collision_detection::Residual>("residual", 2);
  last_grav = jspace::Vector::Zero(7);
  E0 = -10100.0;
  sig_int = 0;

  zero_srv = nh.advertiseService("zero_sigma", &InverseDynamicsSolverWBC::srvZeroSigma, this);
  ROS_INFO("[collision_monitor] Service advertised at r_start_detection");
  
  ROS_DEBUG("wbc inverse dynamics solver initalized (%zu DOF).", ndof);
}

bool InverseDynamicsSolverWBC::srvZeroSigma(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
  sigma = 0;
  return true;
}

void InverseDynamicsSolverWBC::jsCallback(sensor_msgs::JointState::ConstPtr message) {
  j_state = message;
  first_call = true;
}

void InverseDynamicsSolverWBC::updateState(std::vector<double>& pos, 
                                           std::vector<double>& vel, 
                                           std::vector<double>& acc) {


  if (ndof != pos.size())
    ROS_ERROR("dimensions do not agree, expected %zu DOF, pos vector has length %zu.",
              ndof, pos.size());

  if (ndof != vel.size())
    ROS_ERROR("dimensions do not agree, expected %zu DOF, vel vector has length %zu.",
              ndof, vel.size());

  if (ndof != acc.size())
    ROS_ERROR("dimensions do not agree, expected %zu DOF, acc vector has length %zu.",
              ndof, acc.size());

  jspace::convert(&pos[0], ndof, state.position_);
  jspace::convert(&vel[0], ndof, state.velocity_);
  jspace::convert(&acc[0], ndof, q_dotdot);

  model.update(state);
        
}

int JOINTSTATE_INDS_L[] = {29, 30, 28, 32, 31, 33, 34};

bool InverseDynamicsSolverWBC::getTorques(std::vector<double>& torques) {
  ros::spinOnce();
  double delta_time = ros::Time::now().toSec() - last_time;
  last_time = ros::Time::now().toSec();
  if (!model.getMassInertia(M)) {
    ROS_ERROR("jspace::Model::getMassInertia() failed!");
    return false;
  }

  model.getGravity(grav);
  jspace::State s = model.getState();
  double delta[7] = {0.01, 0.001, 0.01, 0.01, 0.01, 0.01, 0.01};
  jspace::Vector K(7);
  K << 90.0, 0.0080, 20.0, 22.0, 7.0, 27.0, 20.0;  
  jspace::Vector p = M * s.velocity_;
  jspace::Matrix M1, M2, DM;
  jspace::Vector alpha = jspace::Vector::Zero(7); //grav;
  jspace::Vector eff = grav;
  collision_detection::Residual res;
  for(int i=0;i<7;i++) {
    s.position_[i] += delta[i];
    if (!model.getMassInertia(M1)) {
      ROS_ERROR("jspace::Model::getMassInertia() failed!");
      return false;
    }
    s.position_[i] -= 2*delta[i];
    if (!model.getMassInertia(M2)) {
      ROS_ERROR("jspace::Model::getMassInertia() failed!");
      return false;
    }
    DM = (M1 - M2) / (2*delta[i]);
    alpha(i) = alpha(i) - 0.5 * (s.velocity_.transpose() * (DM * s.velocity_))(0,0);
    eff(i) = j_state->effort[JOINTSTATE_INDS_L[i]];

    s.position_[i] += delta[i];
  }
  r = K.cwise() * (delta_time * (alpha - eff - r) + p);

  double T = 0.5 * (s.velocity_.transpose() * (M * s.velocity_))(0,0);
  double g_delta = 0.0001;
  jspace::Vector g1, g2, q1, q2;
  for(int i=0;i<7;i++) 
    s.position_(i) += g_delta;
  q1 = s.position_;
  model.getGravity(g1);
  for(int i=0;i<7;i++) 
    s.position_(i) -= 2*g_delta;
  q2 = s.position_;
  model.getGravity(g2);
  for(int i=0;i<7;i++) 
    s.position_(i) += g_delta;

  double U = 0.5 * (g1 + g2).dot(q1 - q2);
  //U = 0.0;
  double E = T + U;
  if(E0 < -10000.0) {
    E0 += 1;
    if(E0 == -10000.0)
      E0 = E;
  }
  double k_sig = 50.0;
  //eff = jspace::Vector::Zero(7);
  sig_int += delta_time * ((s.velocity_.transpose() * eff)(0,0) + sigma);
  sigma = k_sig * (E - sig_int - E0);
  res.data.resize(8);
  for(int i=0;i<7;i++) 
    res.data[i] = r(i);
  res.data[7] = sigma;
  pub.publish(res);
  
  //if(std::abs(sigma) > 1.5)
  //  std::cout << sigma << std::endl;
  //std::cout << E << E0 << std::endl;
  //std::cout << grav.transpose() << std::endl;
  //std::cout << alpha.transpose() << std::endl;
  //std::cout << delta_time << std::endl;
  //std::cout << r.transpose() << std::endl;
  // std::cout << jspace::pretty_string(state.position_) << "   "
  //           << jspace::pretty_string(state.velocity_) << "   "
  //           << jspace::pretty_string(q_dotdot) << "   "
  //           << jspace::pretty_string(tau) << "\n";
  last_grav = grav;

  torques.resize(ndof);
  //jspace::convert(tau, torques);

  return true;
}

size_t InverseDynamicsSolverWBC::getNumDOF() {
  return ndof;
}

// int main(int argc, char ** argv)
// {
//   InverseDynamicsSolverWBC solver = InverseDynamicsSolverWBC("./config/pr2_left_arm_wbc.sai.xml");
//   std::vector<double> pos(7);
//   std::vector<double> vel(7);
//   std::vector<double> acc(7);
//   std::vector<double> torques(7);

//   acc[1] = 0.1;

//   solver.updateState(pos, vel, acc);
//   solver.getTorques(torques);

//   for (std::vector<double>::iterator it = torques.begin(); it != torques.end(); it++)
//     printf("%6.3f ", *it);
//   printf("\n");
// }
