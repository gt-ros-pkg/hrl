/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Daehyung Park (Dr. Charles C. Kemp's HRL, GIT).
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
 *
 * <02/01/2013>
 * This code includes a part of 'simulator_gazebo' package (BSD License)
 * of Willow Garage, Inc.
 * 
 * Daehyung Park is with Dr. Charles C. Kemp's the Healthcare Robotics Lab, 
 * Center for Robotics and Intelligent Machines, Georgia Institute of 
 * Technology. (Contact: deric.park@gatech.edu)
 *
 * We gratefully acknowledge support from DARPA Maximum Mobility
 * and Manipulation (M3) Contract W911NF-11-1-603.
 *********************************************************************/

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/Events.hh>
#include <common/common.hh>
#include <stdio.h>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

namespace gazebo
{   
  class ROSWorldPlugin : public WorldPlugin
  {

    public: ROSWorldPlugin()
    {
      this->world_created_ = false;
    }
    public: ~ROSWorldPlugin()
    {
      // disconnect slots
      event::Events::DisconnectWorldUpdateStart(this->time_update_event_);

      // shutdown ros
      this->rosnode_->shutdown();
      delete this->rosnode_;

    }

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // setup ros related
      if (!ros::isInitialized()){
        std::string name = "ros_world_plugin_node";
        int argc = 0;
        ros::init(argc,NULL,name,ros::init_options::NoSigintHandler);
      }
      else
        ROS_ERROR("Something other than this ros_world plugin started ros::init(...), clock may not be published properly.");

      this->rosnode_ = new ros::NodeHandle("~");

      this->lock_.lock();
      if (this->world_created_)
      {
        this->lock_.unlock();
        return;
      }

      // set flag to true and load this plugin
      this->world_created_ = true;
      this->lock_.unlock();

      this->world = physics::get_world(_parent->GetName());
      if (!this->world)
      {
        ROS_FATAL("cannot load gazebo ros world server plugin, physics::get_world() fails to return world");
        return;
      }

      /// \brief advertise all services
      this->AdvertiseServices();

      // hooks for applying forces, publishing simtime on /clock
      this->time_update_event_   = event::Events::ConnectWorldUpdateStart(boost::bind(&ROSWorldPlugin::publishSimTime,this));
    }

    /// \brief advertise services
    void AdvertiseServices()
    {
      // publish clock for simulated ros time
      this->pub_clock_         = this->rosnode_->advertise<rosgraph_msgs::Clock>("/clock",1/0.0005);

      // set param for use_sim_time if not set by user alread
      this->rosnode_->setParam("/use_sim_time", true);
    }

    void publishSimTime()
    {
      common::Time currentTime = this->world->GetSimTime();
      rosgraph_msgs::Clock ros_time_;
      ros_time_.clock.fromSec(currentTime.Double());
      //  publish time to ros
      this->pub_clock_.publish(ros_time_);
    }

    // 
    private: physics::WorldPtr world;
    private: event::ConnectionPtr time_update_event_;

    /// \brief A mutex to lock access to fields that are used in ROS message callbacks
    private: boost::mutex lock_;

    /// \brief utilites for checking incoming string URDF/XML/Param
    bool world_created_;

    ros::NodeHandle* rosnode_;

    // ROS Publisher & Subscriber
    ros::Publisher pub_clock_;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ROSWorldPlugin)
}

