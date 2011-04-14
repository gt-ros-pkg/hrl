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

#ifndef ID_CONTROLLER_H
#define ID_CONTROLLER_H

#include <vector>
#include <boost/scoped_ptr.hpp>

#include <ros/node_handle.h>
#include <actionlib/server/action_server.h>
#include <filters/filter_chain.h>
#include <pr2_controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

#include <inverse_dynamics/JointState.h>

#include "inverse_dynamics/InverseDynamicsSolverWBC.h"

namespace controller {
  
class IDController: public pr2_controller_interface::Controller {
      
  private:
    pr2_mechanism_model::RobotState *robot_;
    std::vector<pr2_mechanism_model::JointState*> joints_;

    ros::NodeHandle node_;
    ros::Time last_time_;
    std::vector<double> last_vel_;

    boost::scoped_ptr<
      realtime_tools::RealtimePublisher<
      inverse_dynamics::JointState> > controller_state_publisher_;

    filters::MultiChannelFilterChain<double> *pos_filter_;
    filters::MultiChannelFilterChain<double> *vel_filter_;
    filters::MultiChannelFilterChain<double> *acc_filter_;
    filters::MultiChannelFilterChain<double> *eff_filter_;

    InverseDynamicsSolverWBC *solver_;

  public:
    virtual bool init(pr2_mechanism_model::RobotState *robot,
                      ros::NodeHandle &n);
    virtual void starting();
    virtual void update();
    virtual void stopping();
};
  
} 

#endif
