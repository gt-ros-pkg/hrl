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
 * actuator_array_driver.h
 *
 *  Created on: Feb 24, 2011
 *      Author: swilliams8
 */

#ifndef ACTUATOR_ARRAY_DRIVER_H_
#define ACTUATOR_ARRAY_DRIVER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

namespace actuator_array_driver
{

struct JointProperties
{
  std::string joint_name;
  bool has_position_limits;
  double min_position;
  double max_position;
  bool has_velocity_limits;
  double max_velocity;
  bool has_acceleration_limits;
  double max_acceleration;
  bool has_effort_limits;
  double max_effort;
  double friction;
  double damping;
};

class ActuatorArrayDriver
{
public:
  ActuatorArrayDriver();
  virtual ~ActuatorArrayDriver();

  /// \brief helper method that reads/updates the joint states, then publishes the joint state message
  /// This is used internally in spin(), but can also used inside an external run loop
  void read_and_publish();

  /// \brief helper method that puts the node in an infinite read-publish loop. Common practice for driver nodes.
  /// As an alternative to the spin() function, a timer may be set up to periodically call read_and_publish.
  /// This is up to the driver designer to decide.
  void spin();


protected:
  /// \brief ROS command subscription
  ros::Subscriber command_sub_;
  /// \brief ROS command message
  sensor_msgs::JointState command_msg_;
  /// \brief Callback performed upon receipt of a command. Copies the common joint information
  /// between the received command message and the current list of configured joints and stores
  /// it in command_msg_. The pure virtual function command_() is then called.
  void command_callback(const sensor_msgs::JointState::ConstPtr& command_msg);

  /// \brief ROS stop service
  ros::ServiceServer stop_srv_;
  /// \brief ROS stop service callback function. Simply calls the pure virtual stop_() function
  bool stop_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /// \brief ROS home service
  ros::ServiceServer home_srv_;
  /// \brief ROS home service callback function. Simply calls the pure virtual home_() function
  bool home_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /// \brief ROS joint_state publisher
  ros::Publisher joint_state_pub_;
  /// \brief ROS joint_state message
  sensor_msgs::JointState joint_state_msg_;

  /// \brief name of the parameter value that holds the robot description
  std::string robot_description_parameter_;

  /// \brief list of joints to be controlled
  std::vector<JointProperties> joints_;

  /// \brief helper method that performs standard initialization of the
  /// ROS objects (reads parameter server, advertises, subscribes, etc)
  /// This is broken out of the constructor to allow derived classes to choose
  /// how to initialize the system (e.g. Gazebo plugin). Most derived classes
  /// will probably want to call init() in the constructor.
  /// Note: This assumes the joints_ map has already been populated.
  void init(ros::NodeHandle& node);

  /// \brief helper method that parses the URDF contained in the
  /// robot_description_parameter on the Parameter Server.
  /// Only information about the joints already contained the joints_ map
  /// will be updated.
  /// If a joint in the joints_ map does not exist in the URDF, the program will exit
  /// Note: This assumes the joints_ map has already been populated.
  void parse_urdf(const ros::NodeHandle& node);

  /// \brief pure virtual function that handles sensing a command to the device.
  /// This gets called once after every message received on the 'command' topic.
  /// The internal command_msg_ will already be updated with the new command information
  virtual bool command_() = 0;

  /// \brief pure virtual function that sends a stop command to the device.
  /// This gets called in response to a 'stop' service call
  virtual bool stop_() = 0;

  /// \brief pure virtual function that sends the device to the home position.
  /// This gets called in response to a 'home' service call
  virtual bool home_() = 0;

  /// \brief pure virtual function that is responsible for reading the current deivce state
  /// and updating the internal joint_state_msg_
  virtual bool read_(ros::Time ts = ros::Time::now()) = 0;
};

}

#endif /* ACTUATOR_ARRAY_DRIVER_H_ */
