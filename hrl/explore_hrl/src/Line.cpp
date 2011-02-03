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

#include "explore_hrl/Line.h"

#include <iomanip>
//#include <gsl/gsl_fit.h>
//#include <GL/glut.h>
//#include "UnitTest.h"

using namespace std;

namespace geom {

Line::Line():p(0.f,0.f,0.f),d(1.f,0.f,0.f){}
Line::Line(Point p1, Point p2):p(p1){
	d = (p2-p1).normalized();
}
Line::Line(Point p1, Point p2, bool setLen):p(p1){
	d = (p2-p1).normalized();
	length = (p2-p1).length();
}
Line::Line(Point _p, float theta):p(_p){
  d = Point(cos(theta),sin(theta),0.f);
}
//Line::Line(const Line& o):beginDist(o.beginDist),endDist(o.endDist),p(o.p),d(o.d){}

Vector Line::getDirection() const{
	return d;
}

Point Line::getPoint() const{
 	return p;
}

Point Line::eval(float t) const{
	return p + d*t;
}

Point Line::intersect(const Line& o) const {
  float t,s;
  return intersect(o, t, s);
}

Point Line::intersect(const Line& o, float& t, float& s) const{
  Vector u = d;
  Vector v = o.d;
  Vector w = Vector(p - o.p);
  Vector vp = perp(v);
  Vector up = perp(u);
  t = -vp.dot(w) / vp.dot(u);
  s = up.dot(w) / up.dot(v);
  return eval(t);
}

float Line::distFrom(Point p2) const{
	Vector v = p2-p;
	Vector perp = v - d*(d*v);
	return perp.length();
}

bool Line::sameAs(const Line& o, double error) const{
	if(!almostEqual(d,o.d,error) && almostEqual(d,-o.d,error))
		return false;
	return(o.distFrom(p) <= error);
}

bool Line::operator<(const Line& o) const{
	if(p==o.p)
		return d<o.d;
	//else
	return p < o.p;
}

void Line::translate(const Point& trans){
	p+=trans;
}

ostream& Line::output(ostream& out) const{
	return out << p << " + t*" << d;
}

//Drawable* Line::getCopy() const{
//	return new Line(*this);
//}

/*
 * Finds the best line for the set of pts;
 */
//Line Line::getBestFit(const double* xs, const double* ys, uint size){
//	double A, B, cov00, cov01, cov11, resSq;
//	gsl_fit_linear (xs, 1, ys, 1, size, &B, &A, &cov00, &cov01, &cov11, &resSq);
//    if(A<=10){
//    	return Line(Point(0.f,0*A+B,0.f),Point(16,16*A+B,0.f));
//    }
//    else{
//    	gsl_fit_linear (ys, 1, xs, 1, size, &B, &A, &cov00, &cov01, &cov11, &resSq);
//    	return Line(Point(0*A+B,0,0.f),Point(16*A+B,16,0.f));
//    }
//}
//Line Line::getBestFit(const vector<Point>& pts){
//    double* xs = new double[pts.size()];
//    double* ys = new double[pts.size()];
//    for(uint i=0;i<pts.size();i++){
//    	xs[i] = pts[i].x();
//    	ys[i] = pts[i].y();
//    }
//    Line l(Line::getBestFit(xs,ys,pts.size()));
//    delete[] xs;
//    delete[] ys;
//	return l;
//}
//Line Line::getBestFit(const set<Point>& pts){
//    double* xs = new double[pts.size()];
//    double* ys = new double[pts.size()];
//    set<Point>::iterator it = pts.begin();
//    for(int i=0;it!=pts.end();it++,i++){
//    	xs[i] = (*it).x();
//    	ys[i] = (*it).y();
//    }
//    Line l(Line::getBestFit(xs,ys,pts.size()));
//    delete[] xs;
//    delete[] ys;
//    return l;
//}


//void Line::draw() const{
//  glBegin(GL_LINES);
//  glColor3f(0.5, 0.5, 0.5);
//  Point p = eval(0);
//  glVertex2d(p.x,p.y);
//  p = eval(length);
//  glVertex2d(p.x,p.y);
//  glEnd();
//}

//bool Line::test(std::ostream& out){
//	bool result = true;
//	Line l;
//	_UNIT_TEST_(l.p == Point(0.f,0.f));
//	_UNIT_TEST_(l.d == Vector(1.f,0.f));
//	_UNIT_TEST_(l.eval(0.f) == Vector(0.f,0.f));
//	_UNIT_TEST_(l.eval(1.f) == Vector(1.f,0.f));
//	_UNIT_TEST_(l.distFrom(Point(35,26)) == 26);
//	l = Line(Point(0.f,0.f),Point(1.f,1.f));
//	_UNIT_TEST_(l.p == Point(0.f,0.f));
//	_UNIT_TEST_(l.d.almostEqual(Vector(1.f,1.f).normalize()));
//	_UNIT_TEST_(l.eval(0.f) == Point(0.f,0.f));
//	_UNIT_TEST_(l.eval(1.f) == Point(1.f,1.f).normalize());
//	_UNIT_TEST_(Test::doubleEqual(l.distFrom(Point(0,sqrt(2))),1));
//	_UNIT_TEST_(Test::doubleEqual(l.distFrom(Point(0,-sqrt(2))),1));
//
//	_UNIT_TEST_(Line(Point(0,0),Point(0,1)).sameAs(Line(Point(0,0),Point(0,2))));
//	_UNIT_TEST_(Line(Point(0,1),Point(0,2)).sameAs(Line(Point(0,2),Point(0,3))));
//
//	vector<Point> pts;
//	pts.push_back(Point(0,0));
//	pts.push_back(Point(10,10));
//	l = Line::getBestFit(pts);
//	_UNIT_TEST_(l.sameAs(Line(Point(0,0),Point(10,10))));
//
//	pts[1] = Point(0,10);
//	l = Line::getBestFit(pts);
//	_UNIT_TEST_(l.sameAs(Line(Point(0,0),Point(0,10))));
//
//	pts.push_back(Point(0,-10));
//	pts.push_back(Point(0,-.1));
//	pts.push_back(Point(0,.1));
//	l = Line::getBestFit(pts);
//	_UNIT_TEST_(l.sameAs(Line(Point(0,0),Point(0,10))));
//	return result;
//}

ostream& operator<<(ostream& out, const Line& line){
	return line.output(out);
}

}
