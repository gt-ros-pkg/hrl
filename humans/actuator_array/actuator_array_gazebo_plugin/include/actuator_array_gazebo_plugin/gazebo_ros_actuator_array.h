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
 * gazebo_ros_actuator_array.h
 *
 *  Created on: Feb 6, 2011
 *      Author: Stephen Williams
 */

#ifndef GAZEBO_ROS_ACTUATORARRAY_HH
#define GAZEBO_ROS_ACTUATORARRAY_HH

#include <vector>

#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include <gazebo/Body.hh>
#include <gazebo/Joint.hh>

#include "boost/thread/mutex.hpp"
#include <ros/ros.h>
#include <control_toolbox/pid.h>

#define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <ros/subscribe_options.h>
#endif

#include <actuator_array_driver/actuator_array_driver.h>


namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosActuatorArray Interface

 \brief GazeboRosActuatorArray controller.

 This controller requires to a model as its parent. This plugin controls a number of independent
 actuators (probably in some sort of kinematic chain), where each actuator accepts a position
 command. Such things as a Pioneer Arm or Manoi may be modeled with such. Each actuator is controlled
 inside Gazebo with a PID loop (courtesy of control_toolbox::Pid). This PID can be tuned to match
 the performance of the real system. This controller publishes the state of each actuator via
 sensor_msgs::JointState messages to the {robot_namespace}/joint_states topic, and receives joint commands
 via a sensor_msgs::JointState message on the {robot_namespace}/command topic.

 When specifying the controller, each joint that is to be controlled should be placed inside a
 <joint> tag. The joint name (required), home position (optional), and PID parameters (optional) are
 specified inside each <joint> block. See below for an example. Maximum joint efforts (forces/torques),
 and joint velocities are read from the URDF/robot definition.


 Example Usage:
 \verbatim
 <model:physical name="some_fancy_model">
   <controller:gazebo_ros_actuator_array name="actuator_array_controller" plugin="libgazebo_ros_actuator_array.so">
     <alwaysOn>true</alwaysOn>
     <updateRate>30.0</updateRate>
     <robotParam>robot_description</robotParam>
     <joint>
       <name>joint01</name>
       <home>0.0</home>
       <p>10.0</p>
       <i>0.0</i>
       <d>0.0</d>
       <iClamp>0.0</iClamp>
     </joint>
     <joint>
       <name>joint02</name>
       <home>1.57080</home>
       <p>20.0</p>
       <i>0.0</i>
       <d>0.0</d>
       <iClamp>0.0</iClamp>
     </joint>
     <joint>
       <name>joint03</name>
       <home>3.14159</home>
       <p>10.0</p>
       <i>0.0</i>
       <d>2.0</d>
       <iClamp>0.0</iClamp>
     </joint>
   </controller:gazebo_ros_actuator_array>
 </model:physical>
 \endverbatim

 \{
 */

/**

 \brief GazeboRosActuatorArray controller.
 \li Starts a ROS node if none exists.
 \li This controller requires to a model as its parent. This plugin controls a number of independent
 actuators (probably in some sort of kinematic chain), where each actuator accepts a position
 command. Such things as a Pioneer Arm or Manoi may be modeled with such. Each actuator is controlled
 inside Gazebo with a PID loop (courtesy of control_toolbox::Pid). This PID can be tuned to match
 the performance of the real system. This controller publishes the state of each actuator via
 sensor_msgs::JointState messages to the {robot_namespace}/joint_states topic, and receives joint commands
 via a sensor_msgs::JointState message on the {robot_namespace}/command topic.

 \li When specifying the controller, each joint that is to be controlled should be placed inside a
 <joint> tag. The joint name (required), home position (optional), and PID parameters (optional) are
 specified inside each <joint> block. See below for an example. Maximum joint efforts (forces/torques),
 and joint velocities are read from the URDF/robot definition.

 \li Example Usage:
 \verbatim
 <model:physical name="some_fancy_model">
   <controller:gazebo_ros_actuator_array name="actuator_array_controller" plugin="libgazebo_ros_actuator_array.so">
     <alwaysOn>true</alwaysOn>
     <updateRate>30.0</updateRate>
     <robotParam>robot_description</robotParam>
     <joint>
       <name>joint01</name>
       <home>0.0</home>
       <p>10.0</p>
       <i>0.0</i>
       <d>0.0</d>
       <iClamp>0.0</iClamp>
     </joint>
     <joint>
       <name>joint02</name>
       <home>1.57080</home>
       <p>20.0</p>
       <i>0.0</i>
       <d>0.0</d>
       <iClamp>0.0</iClamp>
     </joint>
     <joint>
       <name>joint03</name>
       <home>3.14159</home>
       <p>10.0</p>
       <i>0.0</i>
       <d>2.0</d>
       <iClamp>0.0</iClamp>
     </joint>
   </controller:gazebo_ros_actuator_array>
 </model:physical>
 \endverbatim
 .
 **/

struct GazeboJointProperties : public actuator_array_driver::JointProperties
{
  int joint_index;
  gazebo::Joint* joint_ptr;
  control_toolbox::Pid pid;
  double position;
  double home_position;
};

struct MimicJointProperties : GazeboJointProperties
{
  std::string master_joint_name;
  unsigned int master_joint_index;
  double multiplier;
  double offset;
};

class GazeboRosActuatorArray : public Controller, public actuator_array_driver::ActuatorArrayDriver<GazeboJointProperties>
{
  /// \brief Constructor
public:
  GazeboRosActuatorArray(Entity *parent);
  virtual ~GazeboRosActuatorArray();

protected:
  // Inherited from gazebo::Controller
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:
  /// \brief for setting ROS name space
  ParamT<std::string> *robotNamespaceP;
  std::string robot_namespace_;

  /// \brief for setting the parameter name that holds the robot description
  ParamT<std::string> *robotParamP;

  /// \brief A second set of joint properties for handling Gazebo Mimic joints
  std::map<std::string, MimicJointProperties>  mimic_joints_;

  /// \brief The parent Model
  Model *myParent;

  /// \brief Time of previous update
  gazebo::Time last_time_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  boost::mutex lock_;

#ifdef USE_CBQ
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;
#endif


  void parse_mimic_joints(const ros::NodeHandle& node);
  void update_joint(GazeboJointProperties& gazebo_joint, double command_position, double command_velocity, double command_effort, double dt);


  /// \brief pure virtual function that handles sensing a command to the device.
  /// This gets called once after every message received on the 'command' topic.
  /// The internal command_msg_ will already be updated with the new command information
  bool command_();

  /// \brief pure virtual function that sends a stop command to the device.
  /// This gets called in response to a 'stop' service call
  bool stop_();

  /// \brief pure virtual function that sends the device to the home position.
  /// This gets called in response to a 'home' service call
  bool home_();

  /// \brief pure virtual function that is responsible for reading the current deivce state
  /// and updating the internal joint_state_msg_
  bool read_(ros::Time ts = ros::Time::now());

};

/** \} */
/// @}


}

#endif

