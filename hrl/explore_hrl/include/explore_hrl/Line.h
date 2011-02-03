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

#ifndef _JON_LINE_H_
#define _JON_LINE_H_

#include <iostream>
#include <vector>
#include <set>
#include "Path.h"
#include "Vector.h"

namespace geom {

/*
 * Saves the line as a point and vector pair. This makes it possible to store vertical lines
 * and simplifies some operation such as finding the distance to the line from a point
 */
class Line: public Path {
	private:
		Point p;
		Vector d;
	public:
		Line();
    Line(Point p, float theta);
    Line(Point p1, Point p2);
		Line(Point p1, Point p2, bool setLen);
		Vector getDirection() const;
		Point getPoint() const;
		Point eval(float t) const;
		Point intersect(const Line& o) const;
		Point intersect(const Line& o, float& t, float& s) const;
		float distFrom(Point p) const;
		bool sameAs(const Line& o, double e = EPSILON) const;
		bool operator<(const Line& o) const;
		void translate(const Point& trans);
		std::ostream& output(std::ostream& out) const;

//		Drawable* getCopy() const;

		static Line getBestFit(const std::set<Point>& pts);
		static Line getBestFit(const std::vector<Point>& pts);
		static Line getBestFit(const double* xs, const double* ys, uint size);

		void draw() const;

		static bool test(std::ostream& out);
};

std::ostream& operator<<(std::ostream& out, const Line& line);

}

#endif /*_JON_LINE_H_*/
