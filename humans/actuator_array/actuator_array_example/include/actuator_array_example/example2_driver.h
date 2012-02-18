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
 * This example extends Example1 to use the robot description file. Because the base
 * class was designed to use the robot description information, there are very few
 * code changes. The only different is the custom 'init_actuator_' function now makes
 * use of information onbtained from the robot description instead of using hard-coded
 * defaults. The only extra work really involved is in the creation of the robot
 * description file itself.
 *
 *  Created on: Nov 27, 2011
 *      Author: Stephen Williams
 */

#ifndef EXAMPLE2_DRIVER_H_
#define EXAMPLE2_DRIVER_H_

#include <actuator_array_driver/actuator_array_driver.h>
#include <actuator_array_example/dummy_actuator.h>

namespace actuator_array_example
{
// Create a custom JointProperties struct that additionally holds the home position of each
// actuator and a DummyActuator object
struct Example2JointProperties : public actuator_array_driver::JointProperties
{
  double home;
  DummyActuator actuator;
};

class Example2Driver : public actuator_array_driver::ActuatorArrayDriver<Example2JointProperties>
{
private:

  // Convenience typedef to a map of JointName-JointProperties
  typedef std::map<std::string, Example2JointProperties> JointMap;

  // Keep track of the previous time a read/update was called
  ros::Time previous_time_;

public:
  Example2Driver();
  virtual ~Example2Driver();

  bool init_actuator_(const std::string& joint_name, Example2JointProperties& joint_properties, XmlRpc::XmlRpcValue& joint_data);
  bool command_();
  bool stop_();
  bool home_();
  bool read_(ros::Time ts = ros::Time::now());
};

}
#endif	// EXAMPLE2_DRIVER_H_
