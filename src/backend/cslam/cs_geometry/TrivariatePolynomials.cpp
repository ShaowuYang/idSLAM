/*
 * TrivariatePolynomials.cpp
 *
 *  Created on: Apr 23, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#include "TrivariatePolynomials.h"

namespace cs_geom {

TriPoly2 multTriPoly(const TriPoly1& d, const TriPoly1& q)
{
	TriPoly2 res;

	res[0 ] = d[0]*q[0];
	res[1 ] = d[1]*q[1];
	res[2 ] = d[2]*q[2];
	res[3 ] = d[3]*q[3];

	res[4 ] = d[0]*q[1];
	res[5 ] = d[1]*q[2];
	res[6 ] = d[2]*q[3];
	res[7 ] = d[3]*q[0];

	res[8 ] = d[0]*q[2];
	res[9 ] = d[1]*q[3];
	res[10] = d[2]*q[0];
	res[11] = d[3]*q[1];

	res[12] = d[0]*q[3];
	res[13] = d[1]*q[0];
	res[14] = d[2]*q[1];
	res[15] = d[3]*q[2];

	return res;
}

TriPoly3 multTriPoly(const TriPoly2& p, const TriPoly1& q)
{
	TriPoly3 res;

	res[0 ] = p[0]*q[0];
	res[1 ] = p[1]*q[1];
	res[2 ] = p[2]*q[2];
	res[3 ] = p[3]*q[3];

	res[4 ] = (p[4] + p[13])*q[0] + p[0]*q[1];
	res[5 ] = (p[5] + p[14])*q[1] + p[1]*q[2];
	res[6 ] = (p[6] + p[15])*q[2] + p[2]*q[3];
	res[7 ] = (p[7] + p[12])*q[3] + p[3]*q[0];

	res[8 ] = (p[ 8] + p[10])*q[0] + p[0]*q[2];
	res[9 ] = (p[ 9] + p[11])*q[1] + p[1]*q[3];
	res[10] = (p[10] + p[ 8])*q[2] + p[2]*q[0];
	res[11] = (p[11] + p[ 9])*q[3] + p[3]*q[1];

	res[12] = (p[12] + p[7])*q[0] + p[0]*q[3];
	res[13] = (p[13] + p[4])*q[1] + p[1]*q[0];
	res[14] = (p[14] + p[5])*q[2] + p[2]*q[1];
	res[15] = (p[15] + p[6])*q[3] + p[3]*q[2];

	res[16] = (p[5] + p[14])*q[0] + q[1]*(p[ 8] + p[10]) + q[2]*(p[4] + p[13]);
	res[17] = (p[6] + p[15])*q[1] + q[2]*(p[ 9] + p[11]) + q[3]*(p[5] + p[14]);
	res[18] = (p[7] + p[12])*q[2] + q[3]*(p[10] + p[ 8]) + q[0]*(p[6] + p[15]);
	res[19] = (p[4] + p[13])*q[3] + q[0]*(p[11] + p[ 9]) + q[1]*(p[7] + p[12]);

	return res;
}

} // namespace
