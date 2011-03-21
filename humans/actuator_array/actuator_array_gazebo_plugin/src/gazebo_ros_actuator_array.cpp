/*
 *  Copyright (c) 2011, A.M.Howard, S.Williams
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * gazebo_ros_actuator_array.cpp
 *
 *  Created on: Feb 6, 2011
 *      Author: Stephen Williams
 */

#include <actuator_array_gazebo_plugin/gazebo_ros_actuator_array.h>
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <ros/subscribe_options.h>
#include <angles/angles.h>
#include <urdf/model.h>
#include <limits>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

namespace gazebo
{

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_actuator_array", GazeboRosActuatorArray);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosActuatorArray::GazeboRosActuatorArray(Entity *parent) :
  Controller(parent)
{
  this->myParent = dynamic_cast<Model*> (this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosActuatorArray controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string> ("robotNamespace", "/", 0);
  this->robotParamP = new ParamT<std::string> ("robotParam", "robot_description", 1);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosActuatorArray::~GazeboRosActuatorArray()
{
  delete this->robotNamespaceP;
  delete this->robotParamP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosActuatorArray::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robot_namespace_ = this->robotNamespaceP->GetValue();
  this->robotParamP->Load(node);
  this->robot_description_parameter_ = this->robotParamP->GetValue();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  ros::NodeHandle ros_node = ros::NodeHandle(this->robot_namespace_);


  // Read the list of joints specified in the controller (as child nodes)
  std::string joint_name;
  double kp, ki, kd, imax, imin;
  gazebo::XMLConfigNode *child_node;
  child_node = node->GetChild("joint");
  while (child_node)
  {
    joint_name = child_node->GetString("name", "", 1);
    kp = child_node->GetDouble("p", 1.0, 0);
    ki = child_node->GetDouble("i", 0.0, 0);
    kd = child_node->GetDouble("d", 0.0, 0);
    imax = child_node->GetDouble("iClamp", 0.0, 0);
    imin = -imax;

    // Check that the joint exists
    if (dynamic_cast<gazebo::Joint*> (this->myParent->GetJoint(joint_name)) == NULL)
    {
      ROS_FATAL("GazeboRosActuatorArray plugin error: joint: %s does not exist\n", joint_name.c_str());
    }
    else
    {
      actuator_array_driver::JointProperties joint;
      GazeboJointProperties gazebo_joint;

      joint.joint_name = joint_name;
      gazebo_joint.joint_name = joint_name;
      gazebo_joint.joint_ptr = dynamic_cast<gazebo::Joint*> (this->myParent->GetJoint(joint_name));
      gazebo_joint.pid.setGains(kp, ki, kd, imax, imin);
      gazebo_joint.position = 0.0;
      gazebo_joint.home_position = child_node->GetDouble("home", 0.0, 0);

      // Add joint to list
      this->joints_.push_back(joint);
      this->gazebo_joints_.push_back(gazebo_joint);
    }

    child_node = child_node->GetNext("joint");
  }

  // Read additional properties from the URDF
  parse_urdf(ros_node);

  // read in mimic joint properties
  parse_mimic_joints(ros_node);

  // Resize joint_state, controller_state for the proper number of joints
  unsigned int joint_count = this->joints_.size();
  this->joint_state_msg_.name.resize(joint_count);
  this->joint_state_msg_.position.resize(joint_count);
  this->joint_state_msg_.velocity.resize(joint_count);
  this->joint_state_msg_.effort.resize(joint_count);
  this->command_msg_.name.resize(joint_count);
  this->command_msg_.position.resize(joint_count);
  this->command_msg_.velocity.resize(joint_count);
  this->command_msg_.effort.resize(joint_count);
  for(unsigned int i = 0; i < this->joints_.size(); ++i)
  {
    this->joint_state_msg_.name[i] = this->joints_[i].joint_name;
    this->command_msg_.name[i] = this->joints_[i].joint_name;

    // Copy Home position for each joint into the commanded position
    this->command_msg_.position[i] = this->gazebo_joints_[i].home_position;
  }


  // advertise JointState messages
  ros::AdvertiseOptions joint_state_ao = ros::AdvertiseOptions::create<sensor_msgs::JointState>("joint_states", 1,
                                                                                              ros::SubscriberStatusCallback(),
                                                                                              ros::SubscriberStatusCallback(),
                                                                                              ros::VoidPtr(), &this->queue_);
  this->joint_state_pub_ = ros_node.advertise(joint_state_ao);

  // advertise Home service
  ros::AdvertiseServiceOptions home_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>("home",
                                                                                                boost::bind(&GazeboRosActuatorArray::home_callback, this, _1, _2),
                                                                                                ros::VoidPtr(), &this->queue_);
  this->home_srv_ = ros_node.advertiseService(home_aso);

  // advertise Stop service
  ros::AdvertiseServiceOptions stop_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>("stop",
                                                                                                boost::bind(&GazeboRosActuatorArray::stop_callback, this, _1, _2),
                                                                                                ros::VoidPtr(), &this->queue_);
  this->stop_srv_ = ros_node.advertiseService(stop_aso);

  // subscribe to JointState commands
  ros::SubscribeOptions so = ros::SubscribeOptions::create<sensor_msgs::JointState>("command", 1,
                                                                                    boost::bind(&GazeboRosActuatorArray::command_callback, this, _1),
                                                                                    ros::VoidPtr(), &this->queue_);
  this->command_sub_ = ros_node.subscribe(so);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosActuatorArray::InitChild()
{
  // Get the current time
  this->last_time_ = Simulator::Instance()->GetSimTime();

  // start custom queue
  this->callback_queue_thread_ = new boost::thread(boost::bind(&GazeboRosActuatorArray::QueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosActuatorArray::UpdateChild()
{
  // Get the current time for the message header
  gazebo::Time current_time = Simulator::Instance()->GetSimTime();
  double dt = (current_time - this->last_time_).Double();


  this->lock_.lock();

  // Update the JointState message
  this->joint_state_msg_.header.stamp.sec = current_time.sec;
  this->joint_state_msg_.header.stamp.nsec = current_time.nsec;

  //  Update PID loops, and Copy the state from the gazebo joints into a JointState message
  for (unsigned int i = 0; i < this->joints_.size(); ++i)
  {
    if (!this->gazebo_joints_[i].joint_ptr)
      continue;

    // Update the PID loop and send command to joint
    update_joint(this->joints_[i], this->gazebo_joints_[i], this->command_msg_.position[i], this->command_msg_.velocity[i], this->command_msg_.effort[i], dt);

    // Update the JointState message
    this->joint_state_msg_.position[i] = this->gazebo_joints_[i].position;
    this->joint_state_msg_.velocity[i] = this->gazebo_joints_[i].joint_ptr->GetVelocity(0);
    this->joint_state_msg_.effort[i] = this->gazebo_joints_[i].joint_ptr->GetForce(0);
  }

  // Send commands to Mimic Joints
  for (unsigned int i = 0; i < this->mimic_joints_.size(); ++i)
  {
    if (!this->mimic_gazebo_joints_[i].joint_ptr)
      continue;

    // Calculate the target position based on the master joint
    double command_position = this->mimic_gazebo_joints_[i].multiplier*(this->command_msg_.position[this->mimic_gazebo_joints_[i].master_joint_index] - this->mimic_gazebo_joints_[i].offset);
    double command_velocity = this->mimic_gazebo_joints_[i].multiplier*(this->command_msg_.velocity[this->mimic_gazebo_joints_[i].master_joint_index]);

    // Update the PID loop and send command to joint
    update_joint(this->mimic_joints_[i], this->mimic_gazebo_joints_[i], command_position, command_velocity, 0, dt);
  }


  // Publish joint state
  if(this->joint_state_pub_.getNumSubscribers() > 0)
  {
    this->joint_state_pub_.publish(this->joint_state_msg_);
  }

  this->lock_.unlock();


  // save last time stamp
  this->last_time_ = current_time;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosActuatorArray::FiniChild()
{
  this->queue_.clear();
  this->queue_.disable();
  ros::requestShutdown();
  this->callback_queue_thread_->join();
}

////////////////////////////////////////////////////////////////////////////////
// ???
void GazeboRosActuatorArray::QueueThread()
{
  static const double timeout = 0.01;

  ros::NodeHandle node;
  while (node.ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosActuatorArray::update_joint(actuator_array_driver::JointProperties& joint, GazeboJointProperties& gazebo_joint, double command_position, double command_velocity, double command_effort, double dt)
{
  // Calculate the new joint position and the joint error based on type
  double error = 0.0;
  switch(gazebo_joint.joint_ptr->GetType())
  {
    // Update the joint position based of the joint type
    case gazebo::Joint::HINGE:
    {
      gazebo_joint.position = gazebo_joint.position + angles::shortest_angular_distance(gazebo_joint.position, gazebo_joint.joint_ptr->GetAngle(0).GetAsRadian());
      angles::shortest_angular_distance_with_limits(command_position, gazebo_joint.position, joint.min_position, joint.max_position, error);
      break;
    }
    case gazebo::Joint::SLIDER:
    {
      gazebo_joint.position = gazebo_joint.joint_ptr->GetAngle(0).GetAsRadian();
      error = gazebo_joint.position - command_position;
      break;
    }
    default:
    {
      abort();
    }
  }

  // Update the PID for this joint and get the joint command
  double velocity = gazebo_joint.pid.updatePid(error, ros::Duration(dt));

  // Limit command to max velocity provided in the robot_description
  if(joint.has_velocity_limits)
  {
    if(velocity > joint.max_velocity)
    {
      velocity = joint.max_velocity;
    }
    else if(velocity < -joint.max_velocity)
    {
      velocity = -joint.max_velocity;
    }
  }

  // Limit command velocity to max velocity provided by the command_msg (if any)
  if(command_velocity > 0)
  {
    if(velocity > command_velocity)
    {
      velocity = command_velocity;
    }
    else if(velocity < -command_velocity)
    {
      velocity = -command_velocity;
    }
  }

  // Limit effort to the smaller of the maximum joint effort and the commanded effort
  double effort = std::numeric_limits<float>::max();
  if(joint.has_effort_limits)
  {
    effort = std::min(effort, joint.max_effort);
  }
  if(command_effort > 0)
  {
    effort = std::min(effort, command_effort);
  }

  // Send command to joint
  gazebo_joint.joint_ptr->SetVelocity(0, velocity);
  gazebo_joint.joint_ptr->SetMaxForce(0, effort);
}

void GazeboRosActuatorArray::parse_mimic_joints(const ros::NodeHandle& node)
{
  // Construct a model from the robot description parameter
  std::string full_name = node.resolveName(this->robot_description_parameter_);
  urdf::Model urdf_model;
  while(!node.hasParam(full_name))
  {
    ROS_INFO("ActuatorArrayDriver: waiting for robot_description at %s on the parameter server.", full_name.c_str());
    usleep(100000);
  }
  urdf_model.initParam(full_name);

  // Loop over all joints and check for "mimic" data
  std::map< std::string, boost::shared_ptr< urdf::Joint > >::const_iterator urdf_joint_map;
  for(urdf_joint_map = urdf_model.joints_.begin(); urdf_joint_map != urdf_model.joints_.end(); ++urdf_joint_map)
  {
    // Check if the joint has a "mimic" element
    const boost::shared_ptr< urdf::Joint >& urdf_joint = urdf_joint_map->second;
    if(urdf_joint->mimic)
    {
      // Search for the parent joint in the list of controlled joints
      for(unsigned int i = 0; i < this->joints_.size(); ++i)
      {
        // If the parent joint is a controlled joint, add it to the mimic list
        if((urdf_joint->mimic->joint_name == this->joints_[i].joint_name) && dynamic_cast<gazebo::Joint*> (this->myParent->GetJoint(urdf_joint->name)))
        {
          actuator_array_driver::JointProperties mimic_joint;
          MimicJointProperties mimic_gazebo_joint;

          // Store joint properties from the urdf
          mimic_joint.joint_name = urdf_joint->name;
          if(urdf_joint->limits)
          {
            mimic_joint.min_position = urdf_joint->limits->lower;
            mimic_joint.max_position = urdf_joint->limits->upper;
            mimic_joint.has_position_limits = (mimic_joint.max_position > mimic_joint.min_position);
            mimic_joint.max_velocity = urdf_joint->limits->velocity;
            mimic_joint.has_velocity_limits = (mimic_joint.max_velocity > 0.0);
            mimic_joint.max_effort = urdf_joint->limits->effort;
            mimic_joint.has_effort_limits = (mimic_joint.max_effort > 0.0);
          }

          if(urdf_joint->dynamics)
          {
            mimic_joint.damping = urdf_joint->dynamics->damping;
            mimic_joint.friction = urdf_joint->dynamics->friction;
          }


          // Store additional Gazebo properties
          double kp, ki, kd, imax, imin;
          this->gazebo_joints_[i].pid.getGains (kp, ki, kd, imax, imin);
          mimic_gazebo_joint.joint_name = urdf_joint->name;
          mimic_gazebo_joint.joint_ptr = dynamic_cast<gazebo::Joint*> (this->myParent->GetJoint(urdf_joint->name));
          mimic_gazebo_joint.pid.setGains(kp, ki, kd, imax, imin);
          mimic_gazebo_joint.position = 0.0;
          mimic_gazebo_joint.master_joint_index = i;
          mimic_gazebo_joint.master_joint_name = urdf_joint->mimic->joint_name;
          mimic_gazebo_joint.multiplier = urdf_joint->mimic->multiplier;
          mimic_gazebo_joint.offset = urdf_joint->mimic->offset;


          // Add joint to list
          this->mimic_joints_.push_back(mimic_joint);
          this->mimic_gazebo_joints_.push_back(mimic_gazebo_joint);

        } // end: if
      } // end: for
    } // end: if(urdf_joint->mimic)
  } // end: for(urdf_joint_map

}

bool GazeboRosActuatorArray::command_()
{
  // the Gazebo Update() function actually handles changing the joint positions. Do nothing here.
  return true;
}

bool GazeboRosActuatorArray::stop_()
{
  for(unsigned int i = 0; i < this->joints_.size(); ++i)
  {
    this->command_msg_.name[i] = this->joints_[i].joint_name;
    this->command_msg_.position[i] = this->gazebo_joints_[i].position;
    this->command_msg_.velocity[i] = 0.0;
    this->command_msg_.effort[i] = 0.0;
  }

  return true;
}

bool GazeboRosActuatorArray::home_()
{
  for(unsigned int i = 0; i < this->joints_.size(); ++i)
  {
    this->command_msg_.name[i] = this->joints_[i].joint_name;
    this->command_msg_.position[i] = this->gazebo_joints_[i].home_position;
    this->command_msg_.velocity[i] = 0.0;
    this->command_msg_.effort[i] = 0.0;
  }

  return true;
}

bool GazeboRosActuatorArray::read_(ros::Time ts)
{
  // the Gazebo Update() function actually handles changing the joint positions. Do nothing here.
  return true;
}


}
