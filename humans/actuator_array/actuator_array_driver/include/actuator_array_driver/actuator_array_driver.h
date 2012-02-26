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
#include <urdf/model.h>

namespace actuator_array_driver
{

/*
 * A structure that holds standard information about each joint. If additional information is needed, a derived structure
 * can be provided to the ActuatorArrayDriver base class.
 */
struct JointProperties
{
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

/*
 * This package contains a base class for an Actuator Array Driver. This is intended for use with chains of R/C Servos or other
 * similar devices where position (and velocity) commands are sent at irregular intervals, as opposed to the tight, real-time
 * control loop of the PR2 Controller Manager system.
 *
 * This base class performs some standard functionality, such as parsing joint limits out of the robot_description, subscribing to
 * a 'command' topic, publishing on the 'joint_states' topic, and setting up a 'stop' and 'home' service call. This is designed to
 * work as a stand-alone driver for controlling/tele-operating a chain of servos, or in conjunction with the Actuatory Array
 * Joint Trajectory Action. This class is provided as a convenience only, and is not required for successful operation with the
 * Actuator Array Joint Trajectory Action.
 *
 * The base class is templated on a JOINT structure. If no additional data is needed, the provided JointProperties can be used
 * as the template argument. If additional information needs to be stored on a per-joint basis, then the JointProperties class
 * can be used as a base for a custom JOINT structure.
 */
template<class JOINT>
class ActuatorArrayDriver
{
public:
  ActuatorArrayDriver();
  virtual ~ActuatorArrayDriver();

  /// \brief helper method that reads/updates the joint states, then publishes the joint state message
  /// This is used internally in spin(), but can also be used inside an external run loop
  void read_and_publish();

  /// \brief helper method that puts the node into an infinite read-publish loop. This is a common practice
  /// for driver nodes. As an alternative to the spin() function, a timer may be set up to periodically
  /// call read_and_publish. This is up to the driver designer to decide.
  void spin();


protected:
  /// \brief ROS command subscription
  ros::Subscriber command_sub_;

  /// \brief Holds a copy of the latest ROS command message for each joint
  sensor_msgs::JointState command_msg_;

  /// \brief Callback performed upon receipt of a command. Copies the common joint information
  /// between the received command message and the current list of configured joints and stores
  /// it in command_msg_. The virtual function command_() is then called.
  void command_callback(const sensor_msgs::JointState::ConstPtr& command_msg);

  /// \brief ROS stop service
  ros::ServiceServer stop_srv_;

  /// \brief ROS stop service callback function. Simply calls the virtual stop_() function
  bool stop_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /// \brief ROS home service
  ros::ServiceServer home_srv_;

  /// \brief ROS home service callback function. Simply calls the virtual home_() function
  bool home_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /// \brief ROS joint_state publisher
  ros::Publisher joint_state_pub_;

  /// \brief ROS joint_state message
  sensor_msgs::JointState joint_state_msg_;

  /// \brief name of the parameter value that holds the robot description,
  /// not the robot description itself
  std::string robot_description_parameter_;

  /// \brief The URDF model parsed from the provided robot_description
  urdf::Model urdf_model_;

  /// \brief list of joints to be controlled, accessible by joint name
  std::map<std::string, JOINT> joints_;

  /// \brief helper method that parses the URDF contained in the
  /// robot_description_parameter on the Parameter Server and stores
  /// it in the urdf_model_ property of the base class.
  void parse_urdf(const ros::NodeHandle& node);

  /// \brief helper method that extracts information from the URDF model
  /// about a specific joint name and stores it in the provided 'joint_properties'
  /// struct. If the provided joint name does not exist in the URDF, a warning
  /// will be issued and the 'joint_properties' struct will not be updated.
  void update_joint_from_urdf(const std::string& joint_name, JOINT& joint_properties);

  /// \brief helper method that reads in a list of actuator names from the parameter
  /// server and initializes the joints_ map. This list can be a simple,
  /// comma-separated list loaded via rosparam, or a complex struct defining
  /// additional properties about each actuator. If the struct approach is used,
  /// generally loaded via a YAML file, one entry must be called 'name'. Additionally,
  /// an initialization function named 'init_actuator_' can be implemented. This
  /// function is called once per actuator in the list, and is provided with the
  /// full XMLRPC struct for that actuator. This call occurs after the joint is
  /// updated using the URDF information, so any information inside the URDF can be
  /// accessed by the initialization function. The provided node object should be
  /// in the namespace of the 'joints' list on the parameter server (i.e. a private node)
  void parse_actuator_list(const ros::NodeHandle& node);

  /// \brief helper method that sets up the communication elements with
  /// ROS. This involves subscribing to the 'command' topic, advertising
  /// the 'joint_states' topic, and creating the 'stop' and 'home' services.
  /// All topics will be advertised in the namespace of the provided 'node' object.
  void advertise_and_subscribe(ros::NodeHandle& node);

  /// \brief convenience method that calls 'parse_actuator_list',
  /// 'parse_urdf' and 'advertise_and_subscribe' in order. This
  /// is broken out of the constructor to allow derived classes
  /// to choose how to initialize the system (e.g. Gazebo plugin).
  /// Most derived classes will probably want to call init() in
  /// the constructor.
  void init();

  /// \brief helper method that performs custom initialization on a per-actuator
  /// basis. If the 'joints' parameter contains an XMLRPC array of structs,
  /// then this function is also provided with the full XMLRPC structure. This is
  // a convenience function that allows a single YAML configuration file to both
  /// initialize the list of actuators to control, and define custom properties
  /// about each actuator. Such entries as 'channel id', 'offset' , and 'home'
  /// are common attributes that are unique to each actuator.
  virtual bool init_actuator_(const std::string& joint_name, JOINT& joint_properties, XmlRpc::XmlRpcValue& joint_data) {return true;};

  /// \brief virtual function that is responsible for reading the current device state
  /// and updating the internal joint_state_msg_
  virtual bool read_(ros::Time ts = ros::Time::now()) {return true;};

  /// \brief virtual function that handles sending a command to the device.
  /// This gets called once after every message received on the 'command' topic.
  /// The internal command_msg_ will already be updated with the new command information
  virtual bool command_() {return true;};

  /// \brief virtual function that sends a stop command to the device.
  /// This gets called in response to a 'stop' service call
  virtual bool stop_() {return true;};

  /// \brief virtual function that sends the device to the home position.
  /// This gets called in response to a 'home' service call
  virtual bool home_() {return true;};

};

}

#include <actuator_array_driver/actuator_array_driver_impl.h>

#endif /* ACTUATOR_ARRAY_DRIVER_H_ */
