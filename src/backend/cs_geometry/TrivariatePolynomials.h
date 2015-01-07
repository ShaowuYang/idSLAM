/*
 * TrivariatePolynomials.h
 *
 * Some basic operations on trivariate polynomials
 * (e.g. p1*x + p2*y + *p3*z + p4*1 = 0)
 * required for solving the five point problem.
 *
 *  Created on: Apr 23, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#ifndef _TRIVARIATEPOLYNOMIALS_H_
#define _TRIVARIATEPOLYNOMIALS_H_

#include <cstring> // memset

namespace cs_geom {

// trivariate polynomial of degree 1.
// (p[0]*x + p[1]*y + p[2]*z + p[3])
class TriPoly1 {
public:
	TriPoly1() {}
	virtual ~TriPoly1() {}

	inline double& operator[](int i) { return p[i]; }
	inline const double& operator[](int i) const { return p[i]; }

	inline void zero() { memset(p,0,sizeof(p)); }

	inline TriPoly1 operator+=(const TriPoly1&op) {
		for (int i = 0; i < 4; i++)
			p[i] += op[i];
		return *this;
	};

	// coefficients are stored in this pattern:
	// x y z 1
	double p[4];
};

// trivariate polynomial of degree 2
class TriPoly2 {
public:
	TriPoly2() {}
	virtual ~TriPoly2() {}

	inline double& operator[](int i) { return p[i]; }
	inline const double& operator[](int i) const { return p[i]; }
	inline void zero() { memset(p,0,sizeof(p)); }

	inline TriPoly2 operator+=(const TriPoly2&op) {
		for (int i = 0; i < 16; i++)
			p[i] += op[i];
		return *this;
	};

	inline TriPoly2 operator-=(const TriPoly2&op) {
		for (int i = 0; i < 16; i++)
			p[i] -= op[i];
		return *this;
	};

	inline TriPoly2 operator-(const TriPoly2&op) {
		TriPoly2 res;
		for (int i = 0; i < 16; i++)
			res[i] = p[i] - op[i];
		return res;
	};


	inline TriPoly2 operator*=(const double &op) {
		for (int i = 0; i < 16; i++)
			p[i] *= op;
		return *this;
	};

	// coefficients are stored in this pattern:
	// x^2 y^2 z^2 1
	// x*y y*z z   x
	// x*z y   x*z y
	// x   y*x z*y z
	double p[16];
};

class TriPoly3 {
public:
	TriPoly3() {}
	TriPoly3(const TriPoly3& orig) {
		memcpy(p,orig.p,sizeof(p));
	}
	virtual ~TriPoly3() {}

	inline double& operator[](int i) { return p[i]; }
	inline const double& operator[](int i) const { return p[i]; }
	inline void zero() { memset(p,0,sizeof(p)); }

	inline TriPoly3 operator+=(const TriPoly3&op) {
		for (int i = 0; i < 20; i++)
			p[i] += op[i];
		return *this;
	};

	// coefficients are stored in this pattern:
	// x^3  y^3  z^3  1
	// x^2y y^2z z^2  x
	// x^2z y^2  xz^2 y
	// x^2  xy^2 yz^2 z
	// xyz  yz   xz   xy
	double p[20];
};

TriPoly2 multTriPoly(const TriPoly1& p, const TriPoly1& q);
TriPoly3 multTriPoly(const TriPoly2& p, const TriPoly1& q);

} // namespace


#endif /* _TRIVARIATEPOLYNOMIALS_H_ */
