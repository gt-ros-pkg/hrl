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

#ifndef INVERSEDYNAMICSSOLVERWBC_H
#define INVERSEDYNAMICSSOLVERWBC_H

#include <iostream>
#include <vector>
#include <jspace/Model.hpp>
#include <jspace/test/sai_brep.hpp>
#include <jspace/test/sai_brep_parser.hpp>
#include <ros/ros.h>
#include "inverse_dynamics/InverseDynamicsSolver.h"
#include "inverse_dynamics/Residual.h"
#include "sensor_msgs/JointState.h"

class InverseDynamicsSolverWBC: public InverseDynamicsSolver {

public:
  InverseDynamicsSolverWBC(std::string fname);

  void updateState(std::vector<double>& pos, 
                   std::vector<double>& vel, 
                   std::vector<double>& acc);

  bool getTorques(std::vector<double>& torques);

  size_t getNumDOF();

private:
  size_t ndof;
  jspace::Model model;
  jspace::State state;
  jspace::Matrix M;
  jspace::Vector a;
  jspace::Vector q_dotdot;
  jspace::Vector tau;

  jspace::Vector grav;
  jspace::Vector last_grav;
  jspace::Vector r;
  double sigma;
  double sig_int;
  double E0;
  double last_time;
  ros::NodeHandle nh;
  ros::NodeHandle* nh_priv;
  ros::Publisher pub;
  sensor_msgs::JointState::ConstPtr j_state;
  ros::Subscriber js_sub;
  void jsCallback(sensor_msgs::JointState::ConstPtr message);
  bool first_call;
};

#endif
