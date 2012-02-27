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
 * example3_driver.cpp
 *
 *  Created on: Nov 27, 2011
 *      Author: Stephen Williams
 */

#include <actuator_array_example/example3_driver.h>

namespace actuator_array_example
{

/* ******************************************************** */
Example3Driver::Example3Driver()
{
  // For normal drivers, you would read custom configuration settings from the parameter
  // server here. Common examples include the port name over which you are communicating,
  // or the baud rate for the serial port. For this simple driver, no additional information
  // is required.

  // For example purposes, we are going to call the base class methods manually.

  // Create a private node handle to read parameter settings
  ros::NodeHandle node = ros::NodeHandle();
  ros::NodeHandle private_node = ros::NodeHandle("~");

  // Get the robot_description parameter name from the parameter server
  private_node.param("robot_description_parameter", robot_description_parameter_, std::string("robot_description"));

  // Read in additional joint information from the robot description and store it in the urdf model
  ActuatorArrayDriver::parse_urdf(node);

  // Fill in the joints_ map from the 'joints' list on the parameter server
  ActuatorArrayDriver::parse_actuator_list(private_node);

  // Advertise services and subscribe to topics
  ActuatorArrayDriver::advertise_and_subscribe(node);

}

/* ******************************************************** */
Example3Driver::~Example3Driver()
{
}

/* ******************************************************** */
bool Example3Driver::init_actuator_(const std::string& joint_name, Example3JointProperties& joint_properties, XmlRpc::XmlRpcValue& joint_data)
{
  // Read the additional actuator fields of 'channel' and 'home'
  // from the configuration file data, then create and store a dummy_servo
  // with those parameters

  // Read custom data from the XMLRPC struct
  if (joint_data.hasMember("channel"))
  {
    joint_properties.channel = (int) joint_data["channel"];
  }

  if (joint_data.hasMember("home"))
  {
    joint_properties.home = (double) joint_data["home"];
  }

  // Create a dummy actuator object and store in a container
  actuators_[joint_properties.channel] = DummyActuator(joint_properties.min_position, joint_properties.max_position, joint_properties.max_velocity, joint_properties.home);

  return true;
}

/* ******************************************************** */
bool Example3Driver::read_(ros::Time ts)
{
  // Calculate the time from the last update
  double dt = (ts - previous_time_).toSec();

  // Loop through each joint and request the current status
  for (unsigned int i = 0; i < joint_state_msg_.name.size(); ++i)
  {
    // Look up the channel number from the JointProperties using the joint name
    int channel = joints_[command_msg_.name[i]].channel;

    // Update the simulated state of each actuator by dt seconds
    actuators_[channel].update(dt);

    // Query the current state of each actuator
    joint_state_msg_.position[i] = actuators_[channel].getPosition();
    joint_state_msg_.velocity[i] = actuators_[channel].getVelocity();
    joint_state_msg_.effort[i]   = actuators_[channel].getMaxTorque();
  }

  joint_state_msg_.header.stamp = ts;
  previous_time_ = ts;

  return true;
}

/* ******************************************************** */
bool Example3Driver::command_()
{
  // Loop through each joint in the command message and send the
  // corresponding servo the desired behavior
  for (unsigned int i = 0; i < command_msg_.name.size(); ++i)
  {
    // Look up the channel number from the JointProperties using the joint name
    int channel = joints_[command_msg_.name[i]].channel;

    // Send the actuator the desired position and velocity
    actuators_[channel].setVelocity(command_msg_.velocity[i]);
    actuators_[channel].setPosition(command_msg_.position[i]);
  }

  return true;
}

/* ******************************************************** */
bool Example3Driver::stop_()
{
  // Loop through each joint and send the stop command
  for (unsigned int i = 0; i < command_msg_.name.size(); ++i)
  {
    // Look up the channel number from the JointProperties using the joint name
    int channel = joints_[command_msg_.name[i]].channel;

    // Tell the actuator to stop
    actuators_[channel].stop();
  }

  return true;
}

/* ******************************************************** */
bool Example3Driver::home_()
{
  // Loop through each joint and send the stop command
  for (unsigned int i = 0; i < command_msg_.name.size(); ++i)
  {
    // Look up the channel number from the JointProperties using the joint name
    int channel = joints_[command_msg_.name[i]].channel;

    // Tell the actuator to go to the home position
    actuators_[channel].home();
  }

  return true;
}

} // end namespace actuator_array_example_driver



/* ******************************************************** */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "example3_driver");

  // Create an Example2 Driver Object
  actuator_array_example::Example3Driver driver;

  // Put the driver in an infinite read-publish loop using the base class's 'spin' function
  driver.spin();

  return 0;
}
