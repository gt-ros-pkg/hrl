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
 * example2_driver.cpp
 *
 *  Created on: Nov 27, 2011
 *      Author: Stephen Williams
 */


#include <actuator_array_example/example2_driver.h>

namespace actuator_array_example
{

/* ******************************************************** */
Example2Driver::Example2Driver()
{
  // For normal drivers, you would read custom configuration settings from the parameter
  // server here. Common examples include the port name over which you are communicating,
  // or the baud rate for the serial port. For this very simple example, we don't actually
  // need any additional configuration.

  // For this example, we will call the convenience init() script of the base class. This
  // sets up the list of joints, subscribes to needed topics, advertises services, and
  // calls the custom init_actuator_ function each actuator
  ActuatorArrayDriver::init();

  // The creation of the array of actuators is now performed in the 'init_actuator_' function
}

/* ******************************************************** */
Example2Driver::~Example2Driver()
{
}

/* ******************************************************** */
bool Example2Driver::init_actuator_(const std::string& joint_name, Example2JointProperties& joint_properties, XmlRpc::XmlRpcValue& joint_data){
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
  // Here we are hard coding the min and max joint positions and max velocity.
  // In Example3, this will be read from the robot description. Alternatively,
  // we could have included additional properties in the YAML file and read them
  // as above
  actuators_[joint_properties.channel] = DummyActuator(-1.57, 1.57, 10.0, joint_properties.home);

  return true;
}

/* ******************************************************** */
bool Example2Driver::read_(ros::Time ts)
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
bool Example2Driver::command_()
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
bool Example2Driver::stop_()
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
bool Example2Driver::home_()
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

} // end namespace actuator_array_example



/* ******************************************************** */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "example2_driver");

  // Create an Example2 Driver Object
  actuator_array_example::Example2Driver driver;

  // Put the driver in an infinite read-publish loop using the base class's 'spin' function
  driver.spin();

  return 0;
}
