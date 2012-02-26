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
 * dummy_actuator.h
 *
 *  Created on: Nov 27, 2011
 *      Author: Stephen Williams
 */

#ifndef DUMMY_ACTUATOR_H_
#define DUMMY_ACTUATOR_H_

#include <ros/ros.h>

namespace actuator_array_example
{

// This class simulates a simple RC-type servo motor
class DummyActuator
{
private:

  // Minimum allowed position
  double min_position_;

  // Maximum allowed position
  double max_position_;

  // Maximum allowed velocity
  double max_velocity_;

  // Home position
  double home_;

  // Default velocity
  double default_velocity_;

  // Current Position
  double position_;

  // Current Velocity
  double velocity_;

  // Commanded Position
  double cmd_position_;

  // Commanded Velocity
  double cmd_velocity_;

  // Setup the properties and default values for this actuator
  void configure(double min_position, double max_position, double max_velocity, double home);

public:
  DummyActuator();
  DummyActuator(double min_position, double max_position, double max_velocity, double home);
  virtual ~DummyActuator();

  void update(double dt);
  void setPosition(double position);
  void setVelocity(double velocity);

  double getPosition();
  double getVelocity();
  double getMaxTorque();

  void home();
  void stop();
};

}

#endif	// DUMMY_ACTUATOR_H_
