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

namespace actuator_array_driver
{

/* ******************************************************** */
template<class JOINT>
ActuatorArrayDriver<JOINT>::ActuatorArrayDriver()
{
  return;
}

/* ******************************************************** */
template<class JOINT>
ActuatorArrayDriver<JOINT>::~ActuatorArrayDriver()
{
  return;
}

/* ******************************************************** */
template<class JOINT>
void ActuatorArrayDriver<JOINT>::read_and_publish()
{
  if(read_())
  {
    this->joint_state_pub_.publish(this->joint_state_msg_);
  }
}

/* ******************************************************** */
template<class JOINT>
void ActuatorArrayDriver<JOINT>::spin()
{
  ros::NodeHandle node;
  while (node.ok())
  {
    read_and_publish();
    ros::spinOnce();
  }
}

/* ******************************************************** */
template<class JOINT>
void ActuatorArrayDriver<JOINT>::parse_urdf(const ros::NodeHandle& node)
{
  // Construct a model from the robot description parameter
  std::string full_name = node.resolveName(this->robot_description_parameter_);
  urdf::Model urdf_model;
  while(!node.hasParam(full_name))
  {
    ROS_INFO("ActuatorArrayDriver: waiting for robot_description at %s on the parameter server.", full_name.c_str());
    usleep(100000);
  }
  this->urdf_model_.initParam(full_name);
}

/* ******************************************************** */
template<class JOINT>
void ActuatorArrayDriver<JOINT>::update_joint_from_urdf(const std::string& joint_name, JOINT& joint_properties)
{
  boost::shared_ptr<const urdf::Joint> urdf_joint = this->urdf_model_.getJoint(joint_name);
  if (!urdf_joint)
  {
    ROS_WARN("Joint not found in robot description: %s.", joint_name.c_str());
    return;
  }

  // Store joint properties from a combination of urdf and the parameter server
  if(urdf_joint->limits)
  {
    joint_properties.min_position = urdf_joint->limits->lower;
    joint_properties.max_position = urdf_joint->limits->upper;
    joint_properties.has_position_limits = (joint_properties.max_position > joint_properties.min_position);
    joint_properties.max_velocity = urdf_joint->limits->velocity;
    joint_properties.has_velocity_limits = (joint_properties.max_velocity > 0.0);
    joint_properties.max_effort = urdf_joint->limits->effort;
    joint_properties.has_effort_limits = (joint_properties.max_effort > 0.0);
  }

  if(urdf_joint->dynamics)
  {
    joint_properties.damping = urdf_joint->dynamics->damping;
    joint_properties.friction = urdf_joint->dynamics->friction;
  }
}

/* ******************************************************** */
template<class JOINT>
void ActuatorArrayDriver<JOINT>::parse_actuator_list(const ros::NodeHandle& node)
{
  // Get the list of joints and possibly extended properties from the parameter server
  XmlRpc::XmlRpcValue joints_array;
  node.getParam("joints", joints_array);

  if (joints_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("Parameter 'joints' is not an array");
    return;
  }

  // Resize joint_state, controller_state for the proper number of joints
  unsigned int joint_count = joints_array.size();
  this->joint_state_msg_.name.resize(joint_count);
  this->joint_state_msg_.position.resize(joint_count);
  this->joint_state_msg_.velocity.resize(joint_count);
  this->joint_state_msg_.effort.resize(joint_count);
  this->command_msg_.name.resize(joint_count);
  this->command_msg_.position.resize(joint_count);
  this->command_msg_.velocity.resize(joint_count);
  this->command_msg_.effort.resize(joint_count);

  // Loop over each joint and extract the joint name, parse the URDF file for additional parameters,
  // and call the optional init_actuator_ function.
  for(unsigned int i = 0; i < joint_count; ++i)
  {
    std::string joint_name;
    JOINT joint_properties;
    XmlRpc::XmlRpcValue& joint_data = joints_array[i];
    if(joint_data.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      joint_name = (std::string)joint_data;
    }
    else if(joint_data.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      if (joint_data.hasMember("name"))
      {
        joint_name = (std::string)joint_data["name"];
      }
      else
      {
        std::stringstream ss;
        ss << "Unknown" << std::setw(3) << std::setfill('0') << i;
        joint_name = ss.str();
        ROS_WARN("Parameter 'joints' entry %d does not contain a field 'name'. Using name '%s' instead.", i, joint_name.c_str());
      }
    }

    // Fill in any properties provided by the URDF for this joint
    update_joint_from_urdf(joint_name, joint_properties);

    // Call the optional initialization function for this actuator
    init_actuator_(joint_name, joint_properties, joint_data);

    // Store the completed JointProperties struct in the joints_ map
    this->joints_[joint_name] = joint_properties;

    // Fill in the joint names in the messages
    this->joint_state_msg_.name[i] = joint_name;
    this->command_msg_.name[i] = joint_name;
  }

}

/* ******************************************************** */
template<class JOINT>
void ActuatorArrayDriver<JOINT>::advertise_and_subscribe(ros::NodeHandle& node)
{
  // subscribe to commands in the controller namespace
  this->command_sub_ = node.subscribe("command", 1, &ActuatorArrayDriver::command_callback, this);

  // Advertise services in the controller namespace
  this->stop_srv_ = node.advertiseService("stop", &ActuatorArrayDriver::stop_callback, this);

  // Advertise services in the controller namespace
  this->home_srv_ = node.advertiseService("home", &ActuatorArrayDriver::home_callback, this);

  // Advertise joint states in the parent namespace
  this->joint_state_pub_ = node.advertise<sensor_msgs::JointState>("joint_states", 1);
}

/* ******************************************************** */
template<class JOINT>
void ActuatorArrayDriver<JOINT>::init()
{
  // Create a normal and private node for reading parameters, etc
  ros::NodeHandle node = ros::NodeHandle();
  ros::NodeHandle private_node = ros::NodeHandle("~");

  // Get the robot_description parameter name from the parameter server
  private_node.param("robot_description_parameter", this->robot_description_parameter_, std::string("robot_description"));

  // Read in additional joint information for the robot description
  if(this->robot_description_parameter_.size() > 0)
  {
    parse_urdf(node);
  }

  // Fill in the joints_ map from the 'joints' list on the parameter server
  parse_actuator_list(private_node);

  // Advertise services and subscribe to topics
  advertise_and_subscribe(node);
}

/* ******************************************************** */
template<class JOINT>
void ActuatorArrayDriver<JOINT>::command_callback(const sensor_msgs::JointState::ConstPtr& command_msg)
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
      if(this->command_msg_.name[i] == command_msg->name[j])
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

/* ******************************************************** */
template<class JOINT>
bool ActuatorArrayDriver<JOINT>::stop_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return stop_();
}

/* ******************************************************** */
template<class JOINT>
bool ActuatorArrayDriver<JOINT>::home_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return home_();
}

}
