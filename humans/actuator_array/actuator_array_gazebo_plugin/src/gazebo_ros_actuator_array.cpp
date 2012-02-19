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

#include <limits>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include <angles/angles.h>
#include <urdf/model.h>

#include <actuator_array_gazebo_plugin/gazebo_ros_actuator_array.h>

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
  // This is done instead of using the provided function 'parse_actuator_list' in the base class
  std::string joint_name;
  unsigned int joint_index = 0;
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
      GazeboJointProperties joint;
      joint.joint_index = joint_index++;
      joint.joint_ptr = dynamic_cast<gazebo::Joint*> (this->myParent->GetJoint(joint_name));
      joint.pid.initPid(kp, ki, kd, imax, imin);
      joint.position = 0.0;
      joint.home_position = child_node->GetDouble("home", 0.0, 0);

      // Add joint to list
      this->joints_[joint_name] = joint;
    }

    child_node = child_node->GetNext("joint");
  }

  // Read additional properties from the URDF
  parse_urdf(ros_node);

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
  for(std::map<std::string, GazeboJointProperties>::iterator joint = joints_.begin(); joint != joints_.end(); ++joint)
  {
    const std::string& joint_name = joint->first;
    GazeboJointProperties& joint_properties = joint->second;

    // Fill in any properties provided by the URDF for this joint
    update_joint_from_urdf(joint_name, joint_properties);

    // Copy Home position for each joint into the commanded position
    this->command_msg_.position[joint_properties.joint_index] = joint_properties.home_position;

    // Add joint names to the joint state messages
    this->joint_state_msg_.name[joint_properties.joint_index] = joint_name;
    this->command_msg_.name[joint_properties.joint_index] = joint_name;
  }

  // read in mimic joint properties
  parse_mimic_joints(ros_node);

  // Advertise and subscribe to the required services and topics
  // Because Gazebo uses a custom message queue, this is done instead of using the
  // provided function 'advertise_and_subscribe' in the base class
#ifdef USE_CBQ

  // subscribe to JointState commands
  ros::SubscribeOptions so = ros::SubscribeOptions::create<sensor_msgs::JointState>("command", 1,
                                                                                    boost::bind(&GazeboRosActuatorArray::command_callback, this, _1),
                                                                                    ros::VoidPtr(), &this->queue_);
  this->command_sub_ = ros_node.subscribe(so);

  // advertise Stop service
  ros::AdvertiseServiceOptions stop_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>("stop",
                                                                                                boost::bind(&GazeboRosActuatorArray::stop_callback, this, _1, _2),
                                                                                                ros::VoidPtr(), &this->queue_);
  this->stop_srv_ = ros_node.advertiseService(stop_aso);

  // advertise Home service
  ros::AdvertiseServiceOptions home_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>("home",
                                                                                                boost::bind(&GazeboRosActuatorArray::home_callback, this, _1, _2),
                                                                                                ros::VoidPtr(), &this->queue_);
  this->home_srv_ = ros_node.advertiseService(home_aso);

  // advertise JointState messages
  ros::AdvertiseOptions joint_state_ao = ros::AdvertiseOptions::create<sensor_msgs::JointState>("joint_states", 1,
                                                                                              ros::SubscriberStatusCallback(),
                                                                                              ros::SubscriberStatusCallback(),
                                                                                              ros::VoidPtr(), &this->queue_);
  this->joint_state_pub_ = ros_node.advertise(joint_state_ao);

#else
  this->command_sub_ = ros_node.subscribe<sensor_msgs::JointState>("command", 1, boost::bind(&GazeboRosActuatorArray::command_callback, this, _1));
  this->stop_srv_ = ros_node.advertiseService<std_srvs::Empty>("stop", boost::bind(&GazeboRosActuatorArray::stop_callback, this, _1, _2));
  this->home_srv_ = ros_node.advertiseService<std_srvs::Empty>("home", boost::bind(&GazeboRosActuatorArray::home_callback, this, _1, _2));
  this->joint_state_pub_ = ros_node.advertise<sensor_msgs::JointState>("joint_states", 1);
#endif

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosActuatorArray::InitChild()
{
  // Get the current time
  this->last_time_ = Simulator::Instance()->GetSimTime();

#ifdef USE_CBQ
  // start custom queue
  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosActuatorArray::QueueThread, this));
#endif
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
  for(std::map<std::string, GazeboJointProperties>::iterator joint = this->joints_.begin(); joint != this->joints_.end(); ++joint)
  {
    GazeboJointProperties& joint_properties = joint->second;

    if (!joint_properties.joint_ptr)
      continue;

    // Update the PID loop and send command to joint
    update_joint(joint_properties, this->command_msg_.position[joint_properties.joint_index], this->command_msg_.velocity[joint_properties.joint_index], this->command_msg_.effort[joint_properties.joint_index], dt);

    // Update the JointState message
    this->joint_state_msg_.position[joint_properties.joint_index] = joint_properties.position;
    this->joint_state_msg_.velocity[joint_properties.joint_index] = joint_properties.joint_ptr->GetVelocity(0);
    this->joint_state_msg_.effort[joint_properties.joint_index] = joint_properties.joint_ptr->GetForce(0);
  }

  // Send commands to Mimic Joints
  for(std::map<std::string, MimicJointProperties>::iterator joint = this->mimic_joints_.begin(); joint != this->mimic_joints_.end(); ++joint)
  {
    MimicJointProperties& joint_properties = joint->second;

    if (!joint_properties.joint_ptr)
      continue;

    // Calculate the target position based on the master joint
    double command_position = joint_properties.multiplier*(this->command_msg_.position[joint_properties.master_joint_index] - joint_properties.offset);
    double command_velocity = joint_properties.multiplier*(this->command_msg_.velocity[joint_properties.master_joint_index]);

    // Update the PID loop and send command to joint
    update_joint(joint_properties, command_position, command_velocity, 0, dt);
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
#ifdef USE_CBQ
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
#endif
}

#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// The thread responsible for managing the message queue
void GazeboRosActuatorArray::QueueThread()
{
  static const double timeout = 0.01;

  ros::NodeHandle node;
  while (node.ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

////////////////////////////////////////////////////////////////////////////////
// ???
void GazeboRosActuatorArray::update_joint(GazeboJointProperties& joint, double command_position, double command_velocity, double command_effort, double dt)
{
  // Calculate the new joint position and the joint error based on type
  double error = 0.0;
  switch(joint.joint_ptr->GetType())
  {
    // Update the joint position based of the joint type
    case gazebo::Joint::HINGE:
    {
      joint.position = joint.position + angles::shortest_angular_distance(joint.position, joint.joint_ptr->GetAngle(0).GetAsRadian());
      angles::shortest_angular_distance_with_limits(command_position, joint.position, joint.min_position, joint.max_position, error);
      break;
    }
    case gazebo::Joint::SLIDER:
    {
      joint.position = joint.joint_ptr->GetAngle(0).GetAsRadian();
      error = joint.position - command_position;
      break;
    }
    default:
    {
      abort();
    }
  }

  // Update the PID for this joint and get the joint command
  joint.pid.setCurrentCmd(command_position);
  double velocity = joint.pid.updatePid(error, ros::Duration(dt));

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
  joint.joint_ptr->SetVelocity(0, velocity);
  joint.joint_ptr->SetMaxForce(0, effort);
}

////////////////////////////////////////////////////////////////////////////////
// Gazebo has a special class of joint called a 'Mimic Joint'. This allows one joint
// to control the behavior of one or more mimic joints. This is perhaps most useful
// for situations where a single motion controls multiple links through some sort of
// complex drive train that does not need to be explicitly modeled. Instead, a multiplier
// (similar to gear ratio) is supplied. This function parses the robot description for
// these special Gazebo joints and stores them in a secondary map.
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
      // Find the parent joint
      std::map<std::string, GazeboJointProperties>::iterator parent_joint = joints_.find(urdf_joint->mimic->joint_name);

      // If the parent joint is a controlled joint, add it to the mimic list
      if((parent_joint != joints_.end()) && (dynamic_cast<gazebo::Joint*> (this->myParent->GetJoint(urdf_joint->name))))
      {
        const std::string& parent_name = parent_joint->first;
        GazeboJointProperties& parent_properties = parent_joint->second;

        std::string mimic_name = urdf_joint->name;
        MimicJointProperties mimic_properties;

        // Store joint properties from the urdf
        if(urdf_joint->limits)
        {
          mimic_properties.min_position = urdf_joint->limits->lower;
          mimic_properties.max_position = urdf_joint->limits->upper;
          mimic_properties.has_position_limits = (mimic_properties.max_position > mimic_properties.min_position);
          mimic_properties.max_velocity = urdf_joint->limits->velocity;
          mimic_properties.has_velocity_limits = (mimic_properties.max_velocity > 0.0);
          mimic_properties.max_effort = urdf_joint->limits->effort;
          mimic_properties.has_effort_limits = (mimic_properties.max_effort > 0.0);
        }

        if(urdf_joint->dynamics)
        {
          mimic_properties.damping = urdf_joint->dynamics->damping;
          mimic_properties.friction = urdf_joint->dynamics->friction;
        }

        // Store additional Gazebo properties
        double kp, ki, kd, imax, imin;
        parent_properties.pid.getGains(kp, ki, kd, imax, imin);
        mimic_properties.joint_ptr = dynamic_cast<gazebo::Joint*> (this->myParent->GetJoint(mimic_name));
        mimic_properties.pid.initPid(kp, ki, kd, imax, imin);
        mimic_properties.position = 0.0;
        mimic_properties.master_joint_index = parent_properties.joint_index;
        mimic_properties.master_joint_name = parent_name;
        mimic_properties.multiplier = urdf_joint->mimic->multiplier;
        mimic_properties.offset = urdf_joint->mimic->offset;

        // Add joint to list
        this->mimic_joints_[mimic_name] = mimic_properties;

      } // end: if
    } // end: if(urdf_joint->mimic)
  } // end: for(urdf_joint_map

}

////////////////////////////////////////////////////////////////////////////////
// Function to execute when a command message is received
bool GazeboRosActuatorArray::command_()
{
  // the Gazebo Update() function actually handles changing the joint positions. Do nothing here.
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Function to execute when a 'stop' command is received
bool GazeboRosActuatorArray::stop_()
{
  for(unsigned int i = 0; i < this->command_msg_.name.size(); ++i)
  {
    const GazeboJointProperties& joint_properties = joints_[this->command_msg_.name[i]];

    this->command_msg_.position[i] = joint_properties.position;
    this->command_msg_.velocity[i] = 0.0;
    this->command_msg_.effort[i] = 0.0;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Function to execute when the 'home' command is received
bool GazeboRosActuatorArray::home_()
{
  for(unsigned int i = 0; i < this->command_msg_.name.size(); ++i)
  {
    const GazeboJointProperties& joint_properties = joints_[this->command_msg_.name[i]];

    this->command_msg_.position[i] = joint_properties.home_position;
    this->command_msg_.velocity[i] = 0.0;
    this->command_msg_.effort[i] = 0.0;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Function to execute during every 'read' cycle
bool GazeboRosActuatorArray::read_(ros::Time ts)
{
  // the Gazebo Update() function actually handles changing the joint positions. Do nothing here.
  return true;
}

}
