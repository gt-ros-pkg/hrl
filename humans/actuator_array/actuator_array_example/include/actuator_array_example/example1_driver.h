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
 * This is a very simple example demonstrating the use of the ActuatorArrayDriver
 * base class. This example makes use of convenience classes provided in the base
 * class to read a simple list of actuators from the parameter server and set up
 * the standard ActuatorArrayDriver communication interface. This example does
 * not make use of the robot description. Instead, the init_actuator_ function
 * manually fills in some of the useful properties for each actuator. A
 * DummyActuator class is used to simulate the operation of a real R/C Servo motor.
 * This removes the need to have specific hardware to test the basic
 * ActuatorArrayDriver system.
 *
 *  Created on: Nov 27, 2011
 *      Author: Stephen Williams
 */

#ifndef EXAMPLE1_DRIVER_H_
#define EXAMPLE1_DRIVER_H_

#include <actuator_array_driver/actuator_array_driver.h>
#include <actuator_array_example/dummy_actuator.h>

namespace actuator_array_example
{

class Example1Driver : public actuator_array_driver::ActuatorArrayDriver<actuator_array_driver::JointProperties>
{
private:

  // An array of simulated actuators
  std::map<int, DummyActuator> actuators_;

  // Keep track of the previous time a read/update was called
  ros::Time previous_time_;

public:
  Example1Driver();
  virtual ~Example1Driver();

  // Overload the four main functions used by the ActuatorArrayDriver base class
  bool read_(ros::Time ts = ros::Time::now());
  bool command_();
  bool stop_();
  bool home_();
};

}
#endif	// EXAMPLE1_DRIVER_H_
