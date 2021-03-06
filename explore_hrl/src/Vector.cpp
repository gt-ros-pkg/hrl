/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include "explore_hrl/Vector.h"

namespace geom {

btVector3 perp(const btVector3& v)
{
  return btVector3(-v.y(), v.x(), v.z());
}

bool almostEqual(const Vector& a, const Vector& b, float epsilon) {
  return a.x()-epsilon < b.x() && a.x()+epsilon > b.x() &&
       a.y()-epsilon < b.y() && a.y()+epsilon > b.y();
}

Point getPoint(float theta){
  return Point(cos(theta),sin(theta),0.f);
}

Point getPoint(float r, float theta){
  return Point(r*cos(theta),r*sin(theta),0.f);
}

float getTheta(const Point& p) {
  return atan2(p.y(),p.x());
}

void wrap_angle(float* a) {
  assert(!isnan(*a));
  *a = wrap_angle(*a);
}

float wrap_angle(float a) {
  assert(!isnan(a));
  float q = a + M_PI;
  return wrap_angle2PI(q) - M_PI;
}

void wrap_angle2PI(float* a) {
  assert(!isnan(*a));
  *a = wrap_angle2PI(*a);
}

float wrap_angle2PI(float a) {
  assert(!isnan(a));
  int pies = 2*floor(a/(2*M_PI));
  return a - pies*M_PI;
}

float wrap_angle_0_PI(float a) {
  float res = wrap_angle2PI(a);
  if(res>=M_PI)
    res-=M_PI;
  assert(-EPSILON <= res && res <= M_PI+EPSILON);
  return res;
}

}
