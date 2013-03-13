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
#include <common/common.hh>
#include <angles/angles.h>
#include <stdio.h>

#include "boost/thread/mutex.hpp"

#include "ros/ros.h"
#include "pr2_hardware_interface/hardware_interface.h"
#include "pr2_controller_manager/controller_manager.h"
#include "pr2_gazebo_plugins/SetModelsJointsStates.h"
#include "pr2_mechanism_model/robot.h"
#include "sensor_msgs/JointState.h"
#include <tinyxml.h>

namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {

    public: ROSModelPlugin()
    {
      // Start up ROS
      std::string name = "gazebo_ros_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);
    }
    public: ~ROSModelPlugin()
    {
      this->cm_->~ControllerManager();
      this->rosnode_->shutdown();
      this->ros_spinner_thread_.join();
      delete this->cm_;
      delete this->rosnode_;

      if (this->fake_state_)
        {
          delete this->fake_state_;
        }
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      
      // Get then name of the parent model
      std::string modelName = _sdf->GetParent()->GetValueString("name");

      // Get the world name.
      this->world = _parent->GetWorld();

      // Store the pointer to the model
      this->model = _parent;

      // Error message if the model couldn't be found
      if (!this->model)
        gzerr << "Unable to get parent model\n";

      // get parameter name
      this->robotNamespace = "";
      if (_sdf->HasElement("robotNamespace"))
        this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString();
      
      this->robotParam = "robot_description";
      if (_sdf->HasElement("robotParam"))
        this->robotParam = _sdf->GetElement("robotParam")->GetValueString();
      
      this->robotParam = this->robotNamespace+"/" + this->robotParam;

      // ROS Nodehandle
      this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&ROSModelPlugin::OnUpdate, this));
      
      // pr2_etherCAT calls ros::spin(), we'll thread out one spinner here to mimic that
      this->ros_spinner_thread_ = boost::thread( boost::bind( &ROSModelPlugin::ControllerManagerROSThread,this ) );

      // load a controller manager      
      this->cm_ = new pr2_controller_manager::ControllerManager(&hw_,*this->rosnode_);
      this->hw_.current_time_ = ros::Time(this->world->GetSimTime().Double());
      if (this->hw_.current_time_ < ros::Time(0.001)) this->hw_.current_time_ == ros::Time(0.001); // hardcoded to minimum of 1ms on start up

      this->rosnode_->param("gazebo/start_robot_calibrated",this->fake_calibration_,true);

      // read pr2 urdf
      // setup actuators, then setup mechanism control node
      ReadPr2Xml();

      // Initializes the fake state (for running the transmissions backwards).
      this->fake_state_ = new pr2_mechanism_model::RobotState(&this->cm_->model_);
      
      // The gazebo joints and mechanism joints should match up.
      if (this->cm_->state_ != NULL) // could be NULL if ReadPr2Xml is unsuccessful
        {
          for (unsigned int i = 0; i < this->cm_->state_->joint_states_.size(); ++i)
            {
              std::string joint_name = this->cm_->state_->joint_states_[i].joint_->name;

              // fill in gazebo joints pointer
              gazebo::physics::JointPtr joint = this->model->GetJoint(joint_name);
              if (joint)
                {
                  this->joints_.push_back(joint);
                }
              else
                {
                  this->joints_.push_back(gazebo::physics::JointPtr());  
                  ROS_ERROR("A joint named \"%s\" is not part of Mechanism Controlled joints.\n", joint_name.c_str());
                }
              
            }
        }
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      if (this->world->IsPaused()) return;
      assert(this->joints_.size() == this->fake_state_->joint_states_.size());

      boost::mutex::scoped_lock lock(this->mutex);

      //--------------------------------------------------
      //  Pushes out simulation state
      //--------------------------------------------------    
      // Copies the state from the gazebo joints into the mechanism joints.
      for (unsigned int i = 0; i < this->joints_.size(); ++i)
        {
          if (!this->joints_[i])
            continue;

          this->fake_state_->joint_states_[i].measured_effort_ = this->fake_state_->joint_states_[i].commanded_effort_;
          
          if (this->joints_[i]->HasType(gazebo::physics::Base::HINGE_JOINT))
            {
              gazebo::physics::JointPtr hj = this->joints_[i];
              this->fake_state_->joint_states_[i].position_ = this->fake_state_->joint_states_[i].position_ +
                angles::shortest_angular_distance(this->fake_state_->joint_states_[i].position_,hj->GetAngle(0).Radian());
              this->fake_state_->joint_states_[i].velocity_ = hj->GetVelocity(0);
            }
          else if (this->joints_[i]->HasType(gazebo::physics::Base::SLIDER_JOINT))
            {
              gazebo::physics::JointPtr sj = this->joints_[i];
              {
                this->fake_state_->joint_states_[i].position_ = sj->GetAngle(0).Radian();
                this->fake_state_->joint_states_[i].velocity_ = sj->GetVelocity(0);
              }
            }
          else
            {
            }
        }
      
      // Reverses the transmissions to propagate the joint position into the actuators.
      this->fake_state_->propagateJointPositionToActuatorPosition();

      //--------------------------------------------------
      //  Runs Mechanism Control
      //--------------------------------------------------
      this->hw_.current_time_ = ros::Time(this->world->GetSimTime().Double());
      try
        {
          if (this->cm_->state_ != NULL) // could be NULL if ReadPr2Xml is unsuccessful
            this->cm_->update();
        }
      catch (const char* c)
        {
          if (strcmp(c,"dividebyzero")==0)
            ROS_WARN("pid controller reports divide by zero error");
          else
            ROS_WARN("unknown const char* exception: %s", c);
        }

      //--------------------------------------------------
      //  Takes in actuation commands
      //--------------------------------------------------

      // Reverses the transmissions to propagate the actuator commands into the joints.
      this->fake_state_->propagateActuatorEffortToJointEffort();

      // Copies the commands from the mechanism joints into the gazebo joints.
      for (unsigned int i = 0; i < this->joints_.size(); ++i)
        {
          if (!this->joints_[i])
            continue;
          
          double effort = this->fake_state_->joint_states_[i].commanded_effort_;          
          double damping_coef = 0;
          if (this->cm_->state_ != NULL) // could be NULL if ReadPr2Xml is unsuccessful
            {
              if (this->cm_->state_->joint_states_[i].joint_->dynamics)
                damping_coef = this->cm_->state_->joint_states_[i].joint_->dynamics->damping;
            }

          if (this->joints_[i]->HasType(gazebo::physics::Base::HINGE_JOINT))
            {
              gazebo::physics::JointPtr hj = this->joints_[i];
              double current_velocity = hj->GetVelocity(0);
              double damping_force = damping_coef * current_velocity;
              double effort_command = effort - damping_force;
              hj->SetForce(0,effort_command);
            }
          else if (this->joints_[i]->HasType(gazebo::physics::Base::SLIDER_JOINT))
            {
              gazebo::physics::JointPtr sj = this->joints_[i];
              double current_velocity = sj->GetVelocity(0);
              double damping_force = damping_coef * current_velocity;
              double effort_command = effort-damping_force;
              sj->SetForce(0,effort_command);
            }
        }
    }

    private: bool setModelsJointsStates(pr2_gazebo_plugins::SetModelsJointsStates::Request &req,
                                        pr2_gazebo_plugins::SetModelsJointsStates::Response &res)
    {
      return true;
    }

    private: void ControllerManagerROSThread()
    {
      ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
      
      while (this->rosnode_->ok())
        {
          usleep(1000);
          ros::spinOnce();
        }
    }

    private: void ReadPr2Xml()
    {

      std::string urdf_param_name;
      std::string urdf_string;
      // search and wait for robot_description on param server

      while(urdf_string.empty())
      {
          ROS_DEBUG("gazebo controller manager plugin is waiting for urdf: %s on the param server.", this->robotParam.c_str());
          if (this->rosnode_->searchParam(this->robotParam,urdf_param_name))
            {
              this->rosnode_->getParam(urdf_param_name,urdf_string);
              ROS_DEBUG("found upstream\n%s\n------\n%s\n------\n%s",this->robotParam.c_str(),urdf_param_name.c_str(),urdf_string.c_str());
            }
          else
            {
              this->rosnode_->getParam(this->robotParam,urdf_string);
              ROS_DEBUG("found in node namespace\n%s\n------\n%s\n------\n%s",this->robotParam.c_str(),urdf_param_name.c_str(),urdf_string.c_str());
            }
          usleep(100000);
      }

      ROS_DEBUG("gazebo controller manager got pr2.xml from param server, parsing it...");
      
      // initialize TiXmlDocument doc with a string
      TiXmlDocument doc;
      if (!doc.Parse(urdf_string.c_str()) && doc.Error())
        {
          ROS_ERROR("Could not load the gazebo controller manager plugin's configuration file: %s\n",
                    urdf_string.c_str());
        }
      else
        {
          // Pulls out the list of actuators used in the robot configuration.
          struct GetActuators : public TiXmlVisitor
          {
            std::set<std::string> actuators;
            virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
            {
              if (elt.ValueStr() == std::string("actuator") && elt.Attribute("name"))
                actuators.insert(elt.Attribute("name"));
              else if (elt.ValueStr() == std::string("rightActuator") && elt.Attribute("name"))
                actuators.insert(elt.Attribute("name"));
              else if (elt.ValueStr() == std::string("leftActuator") && elt.Attribute("name"))
                actuators.insert(elt.Attribute("name"));
              return true;
            }
          } get_actuators;
          doc.RootElement()->Accept(&get_actuators);
          
          // Places the found actuators into the hardware interface.
          std::set<std::string>::iterator it;
          for (it = get_actuators.actuators.begin(); it != get_actuators.actuators.end(); ++it)
            {
              pr2_hardware_interface::Actuator* pr2_actuator = new pr2_hardware_interface::Actuator(*it);
              pr2_actuator->state_.is_enabled_ = true;
              this->hw_.addActuator(pr2_actuator);
            }
          
          // Setup mechanism control node
          this->cm_->initXml(doc.RootElement());

          for (unsigned int i = 0; i < this->cm_->state_->joint_states_.size(); ++i)
            this->cm_->state_->joint_states_[i].calibrated_ = fake_calibration_;
        }
    }

    /////////////////////////////////////////////////////////
    // Pointer to the model
    private: physics::ModelPtr model;
  
    // Pointer to the joints
    std::vector<gazebo::physics::JointPtr>  joints_;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* rosnode_;
    
    // PR2 Controller
    pr2_hardware_interface::HardwareInterface hw_;
    pr2_controller_manager::ControllerManager *cm_;
    pr2_mechanism_model::RobotState *fake_state_;

    private: ros::ServiceServer setModelsJointsStatesService;
  
    // ROS Parameter
    std::string robotParam;
    std::string robotNamespace;
    bool fake_calibration_;
    
    //private: void ControllerManagerROSThread();
    private: boost::thread ros_spinner_thread_;

    // Pointer to the model
    private: physics::WorldPtr world;

    // subscribe to world stats
    private: transport::NodePtr node;
    private: transport::SubscriberPtr statsSub;
    //private: common::Time simTime;

    /// Mutex to protect updates.
    private: boost::mutex mutex;
    //private: common::Time last_time_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
