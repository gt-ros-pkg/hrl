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

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "inverse_dynamics/InverseDynamicsSolverWBC.h"

int main( int argc, char** argv )
{
  if (argc != 2) {
    printf("define input filename!\n");
    exit(-1);
  }

  std::string infname(argv[1]);
  std::ifstream infile;
  infile.open(infname.c_str());
  if (!infile) {
    printf("couldn't load %s!\n", infname.c_str());
    exit(-1);
  }
  std::istream* is = &infile;

  std::string sai_xml_fname("./config/pr2_left_arm_wbc.sai.xml");

  std::vector<std::string> joint_names;
  joint_names.push_back("r_shoulder_pan_joint");
  joint_names.push_back("r_shoulder_lift_joint");
  joint_names.push_back("r_upper_arm_roll_joint");
  joint_names.push_back("r_elbow_flex_joint");
  joint_names.push_back("r_forearm_roll_joint");
  joint_names.push_back("r_wrist_flex_joint");
  joint_names.push_back("r_wrist_roll_joint");

  InverseDynamicsSolverWBC solver = 
    InverseDynamicsSolverWBC(sai_xml_fname);

  const size_t ndof = solver.getNumDOF();
  const size_t ndof2(2 * ndof);
  const size_t ndof3(3 * ndof);
  for (size_t lineno(1); *is; ++lineno) {
    std::vector<double> array;
    std::string line;
    getline(*is, line);
    std::istringstream ls(line);
    double value;
    while (ls >> value) {
      array.push_back(value);
    }

    if (array.size() == 0)
      break;

    if (ndof3 != array.size()) {
      printf("expected %zu values, got %zu!\n", ndof3, array.size());
      exit(-1);
    }

    std::vector<double> pos(ndof);
    std::vector<double> vel(ndof);
    std::vector<double> acc(ndof);

    std::copy(array.begin(), array.begin() + ndof, pos.begin());
    std::copy(array.begin() + ndof + 1, array.begin() + ndof2, vel.begin());
    std::copy(array.begin() + ndof2 + 1, array.end(), acc.begin());

    std::vector<double> tau(ndof);
    solver.updateState(pos, vel, acc);
    solver.getTorques(tau);

    for (std::vector<double>::iterator it = tau.begin(); it < tau.end(); it++) {
      std::cout << *it << " ";
    }
    std::cout << "\n";

  }
}
