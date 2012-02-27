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
 * In this example we will set parameters for each servo based on information stored
 * in the URDF robot description. Because the base class was designed to use the robot
 * description information, there are very few code changes. The only difference
 * is the custom 'init_actuator_' function now makes use of information obtained
 * from the robot description instead of using hard-coded defaults. The only extra
 * work really involved is in the creation of the robot description file itself.
 *
 * Additionally, we will explore the initialization process in detail. In the first
 * two examples, the convenience function 'init()' was used. Here we will perform
 * initialization by manually calling helper functions provided by the base class.
 *
 *  Created on: Nov 27, 2011
 *      Author: Stephen Williams
 */

#ifndef EXAMPLE3_DRIVER_H_
#define EXAMPLE3_DRIVER_H_

#include <actuator_array_driver/actuator_array_driver.h>
#include <actuator_array_example/dummy_actuator.h>

namespace actuator_array_example
{

struct Example3JointProperties : public actuator_array_driver::JointProperties
{
  int channel;
  double home;
};

class Example3Driver : public actuator_array_driver::ActuatorArrayDriver<Example3JointProperties>
{
private:

  // A container of DummyActuator objects, stored by Channel ID
  std::map<int, DummyActuator> actuators_;

  // Keep track of the previous time a read/update was called
  ros::Time previous_time_;

public:
  Example3Driver();
  virtual ~Example3Driver();

  bool init_actuator_(const std::string& joint_name, Example3JointProperties& joint_properties, XmlRpc::XmlRpcValue& joint_data);
  bool read_(ros::Time ts = ros::Time::now());
  bool command_();
  bool stop_();
  bool home_();
};

}
#endif	// EXAMPLE3_DRIVER_H_
