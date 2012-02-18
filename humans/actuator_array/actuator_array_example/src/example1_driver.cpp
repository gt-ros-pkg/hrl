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
 * example1_driver.cpp
 *
 *  Created on: Nov 27, 2011
 *      Author: Stephen Williams
 */

#include <actuator_array_example/example1_driver.h>

namespace actuator_array_example
{

/* ******************************************************** */
Example1Driver::Example1Driver()
{
  // For normal drivers, you would read custom configuration settings from the parameter
  // server here. Common examples include the port name over which you are communicating,
  // or the baud rate for the serial port. For this very simple example, we don't actually
  // need any additional configuration.

  // For this first, simple example, we will call the convenience init() script of the
  // base class. This sets up the list of joints, subscribes to needed topics, advertises
  // services, and calls the custom init_actuator_ function each actuator
  ActuatorArrayDriver::init();
}

/* ******************************************************** */
Example1Driver::~Example1Driver()
{
}

/* ******************************************************** */
bool Example1Driver::init_actuator_(const std::string& joint_name, Example1JointProperties& joint_properties, XmlRpc::XmlRpcValue& joint_data){

  // Since this is a simple example, use a set of default properties for all joints
  joint_properties.has_position_limits = true;
  joint_properties.min_position = -1.57;
  joint_properties.max_position = 1.57;
  joint_properties.has_velocity_limits = true;
  joint_properties.max_velocity = 10.0;
  joint_properties.home = 0.0;
  joint_properties.actuator = DummyActuator(joint_properties.min_position, joint_properties.max_position, joint_properties.max_velocity, joint_properties.home);

  return true;
}

/* ******************************************************** */
bool Example1Driver::command_()
{
  // Loop through each joint in the command message and send the
  // corresponding servo the desired behavior
  for (unsigned int i = 0; i < this->command_msg_.name.size(); ++i)
  {
    // Get a reference to the joints_ entry for this joint
    Example1JointProperties& joint_properties = joints_[this->command_msg_.name[i]];

    joint_properties.actuator.setVelocity(this->command_msg_.velocity[i]);
    joint_properties.actuator.setPosition(this->command_msg_.position[i]);
  }

  return true;
}

/* ******************************************************** */
bool Example1Driver::stop_()
{
  // Loop through each joint and send the stop command
  for (unsigned int i = 0; i < this->command_msg_.name.size(); ++i)
  {
    // Get a reference to the joints_ entry for this joint
    Example1JointProperties& joint_properties = joints_[this->command_msg_.name[i]];

    // Update the simulated state of each actuator by dt seconds
    joint_properties.actuator.stop();
  }

  return true;
}

/* ******************************************************** */
bool Example1Driver::home_()
{
  // Loop through each joint and send the stop command
  for (unsigned int i = 0; i < this->command_msg_.name.size(); ++i)
  {
    // Get a reference to the joints_ entry for this joint
    Example1JointProperties& joint_properties = joints_[this->command_msg_.name[i]];

    // Update the simulated state of each actuator by dt seconds
    joint_properties.actuator.home();
  }

  return true;
}

/* ******************************************************** */
bool Example1Driver::read_(ros::Time ts)
{
  // Calculate the time from the last update
  double dt = (ts - this->previous_time_).toSec();

  // Loop through each joint and request the current status
  for (unsigned int i = 0; i < this->joint_state_msg_.name.size(); ++i)
  {
    // Get a reference to the joints_ entry for this joint
    Example1JointProperties& joint_properties = joints_[this->joint_state_msg_.name[i]];

    // Update the simulated state of each actuator by dt seconds
    joint_properties.actuator.update(dt);

    // Query the current state of each actuator
    this->joint_state_msg_.position[i] = joint_properties.actuator.getPosition();
    this->joint_state_msg_.velocity[i] = joint_properties.actuator.getVelocity();
    this->joint_state_msg_.effort[i]   = joint_properties.actuator.getMaxTorque();
  }

  this->joint_state_msg_.header.stamp = ts;
  this->previous_time_ = ts;

  return true;
}

} // end namespace actuator_array_example



/* ******************************************************** */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "example1_driver");

  // Create an Example1 Driver Object
  actuator_array_example::Example1Driver driver;

  // Put the driver in an infinite read-publish loop using the base class's 'spin' function
  driver.spin();

  return 0;
}
