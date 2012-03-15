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

#include <iostream>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "kdl_dynamics");

  KDL::Tree kdl_tree;
  ros::NodeHandle node;
  std::string robot_desc_string;
  node.param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
    ROS_ERROR("Failed to construct KDL tree.");
    exit(-1);
  }

  // std::cout << kdl_tree.getNrOfJoints() << std::endl;
  // KDL::SegmentMap kdl_segments = kdl_tree.getSegments();
  // for (KDL::SegmentMap::iterator it = kdl_segments.begin(); 
  //      it != kdl_segments.end(); 
  //      ++it) {
  //   std::cout << (*it).first << std::endl;
  // }

  KDL::Chain kdl_chain;
  if (!kdl_tree.getChain("torso_lift_link", "r_wrist_roll_link", kdl_chain)) {
    ROS_ERROR("Failed to get chain from KDL tree.");
    exit(-1);
  }

  KDL::Vector grav;
  KDL::ChainIdSolver_RNE kdl_idsolver(kdl_chain, grav);
  
  std::istream * in;
  in = &std::cin;
  
  const size_t ndof(kdl_chain.getNrOfJoints());
  const size_t nstate(3 * ndof);

  int status = 0;

  for (size_t lineno(1); *in; ++lineno) {
    std::string line;
    getline(*in, line);

    std::vector<double> state;
    std::istringstream ls(line);
    double value;
    while (ls >> value) {
      state.push_back(value);
    }

    // end of file or input
    if (0 == state.size()) {
      break;
    }

    // check if input dimensions agree
    if (nstate != state.size()) {
      ROS_ERROR("expected %zu DOF, but read line with %zu (!=%zu) doubles!", ndof, state.size(), nstate);
      break;
    }

    // compute torques
    KDL::JntArray q = KDL::JntArray(ndof);
    KDL::JntArray q_dot = KDL::JntArray(ndof);
    KDL::JntArray q_dotdot = KDL::JntArray(ndof);
    KDL::JntArray torques = KDL::JntArray(ndof);
    KDL::Wrenches f_ext;
    f_ext.resize(ndof);

    status = kdl_idsolver.CartToJnt(q, q_dot, q_dotdot, f_ext, torques);
    if (status < 0)
      ROS_ERROR("KDL inverse dynamics solver returned: %d", status);

    for (uint i = 0; i < ndof; i++) {
      std::cout << torques(i) << " ";
    }
    std::cout << std::endl;
 
  }

}
