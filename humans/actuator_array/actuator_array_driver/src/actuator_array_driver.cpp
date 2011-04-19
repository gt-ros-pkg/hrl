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
 * actuator_array_driver.cpp
 *
 *  Created on: Feb 24, 2011
 *      Author: swilliams8
 */

#include <actuator_array_driver/actuator_array_driver.h>
#include <urdf/model.h>

namespace actuator_array_driver
{

ActuatorArrayDriver::ActuatorArrayDriver()
{
  return;
}

ActuatorArrayDriver::~ActuatorArrayDriver()
{
  return;
}

void ActuatorArrayDriver::command_callback(const sensor_msgs::JointState::ConstPtr& command_msg)
{
  // sensor_msgs::JointState definition specifies that each vector can either be empty, or contain the
  // same number of elements as "name". Check for empty vectors.
  bool has_position = command_msg->position.size();
  bool has_velocity = command_msg->velocity.size();
  bool has_effort = command_msg->effort.size();

  // Create a mapping between the JointState command message joint indices and the class joint indices
  // The internal command information will be updated with information from any matching joints
  for(unsigned int i = 0; i < this->command_msg_.name.size(); ++i)
  {
    for(unsigned int j = 0; j < command_msg->name.size(); ++j)
    {
      if(command_msg->name[j] == this->command_msg_.name[i])
      {
        if(has_position)
        {
          this->command_msg_.position[i] = command_msg->position[j];
        }
        if(has_velocity)
        {
          this->command_msg_.velocity[i] = command_msg->velocity[j];
        }
        if(has_effort)
        {
          this->command_msg_.effort[i] = command_msg->effort[j];
        }
        this->command_msg_.header = command_msg->header;
        break;
      }
    }
  }

  // Call pure virtual command_ function to do something useful
  command_();
}

bool ActuatorArrayDriver::stop_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return stop_();
}

bool ActuatorArrayDriver::home_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return home_();
}

void ActuatorArrayDriver::init(ros::NodeHandle& node)
{
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
  std::vector<JointProperties >::iterator joint;
  unsigned int i;
  for(i = 0, joint = this->joints_.begin(); joint != this->joints_.end(); ++joint, ++i)
  {
    this->joint_state_msg_.name[i] = joint->joint_name;
    this->command_msg_.name[i] = joint->joint_name;
  }

  // Create a private node for reading parameters, etc
  ros::NodeHandle private_node = ros::NodeHandle("~");

  // Get the robot_description parameter name from the parameter server
  private_node.param("robot_description", this->robot_description_parameter_, std::string("robot_description"));

  // Update the joint properties using the robot_description
  parse_urdf(node);


  // subscribe to commands in the controller namespace
  this->command_sub_ = node.subscribe("command", 1, &ActuatorArrayDriver::command_callback, this);

  // Advertise services in the controller namespace
  this->stop_srv_ = node.advertiseService("stop", &ActuatorArrayDriver::stop_callback, this);

  // Advertise services in the controller namespace
  this->home_srv_ = node.advertiseService("home", &ActuatorArrayDriver::home_callback, this);

  // Advertise joint states in the parent namespace
  this->joint_state_pub_ = node.advertise<sensor_msgs::JointState>("joint_states", 1);

}

void ActuatorArrayDriver::parse_urdf(const ros::NodeHandle& node)
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

  std::vector<JointProperties>::iterator joint;
  for(joint = this->joints_.begin(); joint != this->joints_.end(); ++joint)
  {
    boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model.getJoint(joint->joint_name);
    if (!urdf_joint)
    {
      ROS_ERROR("Joint not found in robot description: %s.", joint->joint_name.c_str());
      exit(1);
    }

    // Store joint properties from a combination of urdf and the parameter server
    if(urdf_joint->limits)
    {
      joint->min_position = urdf_joint->limits->lower;
      joint->max_position = urdf_joint->limits->upper;
      joint->has_position_limits = (joint->max_position > joint->min_position);
      joint->max_velocity = urdf_joint->limits->velocity;
      joint->has_velocity_limits = (joint->max_velocity > 0.0);
      joint->max_effort = urdf_joint->limits->effort;
      joint->has_effort_limits = (joint->max_effort > 0.0);
    }

    if(urdf_joint->dynamics)
    {
      joint->damping = urdf_joint->dynamics->damping;
      joint->friction = urdf_joint->dynamics->friction;
    }

  }

}

void ActuatorArrayDriver::read_and_publish()
{
  if(read_())
  {
    this->joint_state_pub_.publish(this->joint_state_msg_);
  }
}

void ActuatorArrayDriver::spin()
{
  ros::NodeHandle node;
  while (node.ok())
  {
    read_and_publish();
    ros::spinOnce();
  }
}


}
