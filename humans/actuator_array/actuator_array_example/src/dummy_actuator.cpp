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
 * dummy_actuator.cpp
 *
 *  Created on: Nov 27, 2011
 *      Author: Stephen Williams
 */

#include <actuator_array_example/dummy_actuator.h>

namespace actuator_array_example
{

int sgn(double val)
{
    return (val > double(0)) - (val < double(0));
}

DummyActuator::DummyActuator()
{
  configure(-1.57, 1.57, 10.0, 0.0);
}

DummyActuator::DummyActuator(double min_position, double max_position, double max_velocity, double home)
{
  configure(min_position, max_position, max_velocity, home);
}

DummyActuator::~DummyActuator()
{
}

void DummyActuator::configure(double min_position, double max_position, double max_velocity, double home)
{
  min_position_ = min_position;
  max_position_ = max_position;
  max_velocity_ = max_velocity;
  home_ = home;

  default_velocity_ = max_velocity / 2.0;
  position_ = home;
  velocity_ = 0.0;
  cmd_position_ = home;
  cmd_velocity_ = default_velocity_;
}

void DummyActuator::update(double dt)
{
  double position_error = cmd_position_ - position_;
  if(fabs(position_error) < 0.01)
  {
    velocity_ = 0.0;
  }
  else if(fabs(position_error) < dt*cmd_velocity_)
  {
    velocity_ = fabs(position_error) / dt;
  }
  else
  {
    velocity_ = cmd_velocity_;
  }

  position_ += velocity_ * dt * sgn(position_error);
}

void DummyActuator::setPosition(double position)
{
  if(position < min_position_)
  {
    cmd_position_ = min_position_;
  }
  else if(position > max_position_)
  {
    cmd_position_ = max_position_;
  }
  else
  {
    cmd_position_ = position;
  }

  if(cmd_velocity_ <= 0.0)
  {
    cmd_velocity_ = default_velocity_;
  }
}

void DummyActuator::setVelocity(double velocity)
{
  cmd_velocity_ = fabs(velocity);
  if(cmd_velocity_ > max_velocity_)
  {
    cmd_velocity_ = max_velocity_;
  }
}

double DummyActuator::getPosition()
{
  return position_;
}

double DummyActuator::getVelocity()
{
  return velocity_;
}

double DummyActuator::getMaxTorque()
{
  return 0.0;
}

void DummyActuator::home()
{
  cmd_position_ = home_;
  cmd_velocity_ = default_velocity_;
}

void DummyActuator::stop()
{
  cmd_position_ = position_;
  cmd_velocity_ = 0.0;
}

} // actuator_array_example
