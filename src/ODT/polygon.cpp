#include "polygon.h"
using namespace ptam;
using namespace std;
/*
 * modified C++ cide from C code from the article
 * "Testing the Convexity of a Polygon"
 * by Peter Schorn and Frederick Fisher,
 *	(schorn@inf.ethz.ch, fred@kpc.com)
 * in "Graphics Gems IV", Academic Press, 1994

 // http://erich.realtimerendering.com/ptinpoly/#ref9
 */
/* Program to Classify a Polygon's Shape */

/* Classify the polygon vertices formed by points f according to: 'NotConvex'	*/
/* 'NotConvexDegenerate', 'ConvexDegenerate', 'ConvexCCW', 'ConvexCW'.	*/
// @param f the corners formming the polygon
// @param n number of the corners
PolygonClass Polygon::ClassifyPolygon(vector<cv::Point> f, int n)
{
    int		 curDir, thisDir, thisSign, angleSign = 0, dirChanges = 0;
    cv::Point	 first, second, third, saveFirst, saveSecond;

    int point_num = 0;// the number of element which should be read by GetPoint.
    if ( !GetPoint(f, point_num, &first) || !GetDifferentPoint(f, point_num, first, &second) )
        return ConvexDegenerate;
    saveFirst = first;	saveSecond = second;
    curDir = Compare(first, second);
    while( GetDifferentPoint(f, point_num, second, &third) ) {
        // in my landing pad detection case, repeated corners are not allowed!
        if (!Compare(saveFirst, third))   return  ConvexDegenerate;
        /////////////////

        CheckTriple;
    }
    /* Must check that end of list continues back to start properly */
    if ( Compare(second, saveFirst) ) {
    third = saveFirst; CheckTriple;
    }
    third = saveSecond;	 CheckTriple;

    if ( dirChanges > 2 ) return angleSign ? NotConvex : NotConvexDegenerate;
    if ( angleSign  > 0 ) return ConvexCCW;
    if ( angleSign  < 0 ) return ConvexCW;
    return ConvexDegenerate;
}

// http://geomalgorithms.com/a03-_inclusion.html
// winding number algorithm for the inclusion of a point in polygon
// Copyright 2000 softSurfer, 2012 Dan Sunday
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.

// But the bottom line is that for both geometric correctness and efficiency reasons,
// the wn algorithm should always be preferred for determining the inclusion of a
// point in a polygon.

// a Point is defined by its coordinates {int x, y;}
//===================================================================

// cn_PnPoly(): crossing number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  0 = outside, 1 = inside
// This code is patterned after [Franklin, 2000]
int Polygon::cn_PnPoly( cv::Point P, vector<cv::Point> V, int n )
{
    V.push_back(V[0]);
    assert(V.size() == n+1);
    int    cn = 0;    // the  crossing number counter

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {    // edge from V[i]  to V[i+1]
       if (((V[i].y <= P.y) && (V[i+1].y > P.y))     // an upward crossing
        || ((V[i].y > P.y) && (V[i+1].y <=  P.y))) { // a downward crossing
            // compute  the actual edge-ray intersect x-coordinate
            float vt = (float)(P.y  - V[i].y) / (V[i+1].y - V[i].y);
            if (P.x <  (V[i].x + vt * (V[i+1].x - V[i].x))) // P.x < intersect
                 cn++;   // a valid crossing of y=P.y right of P.x
        }
    }
    return (cn&1);    // 0 if even (out), and 1 if  odd (in)

}
//===================================================================
// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only when P is outside)
int Polygon::wn_PnPoly( cv::Point P, vector<cv::Point> V, int n )
{
    V.push_back(V[0]);
    assert(V.size() == n+1);
    int    wn = 0;    // the  winding number counter

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {   // edge from V[i] to  V[i+1]
        if (V[i].y <= P.y) {          // start y <= P.y
            if (V[i+1].y  > P.y){      // an upward crossing
                 if (isLeft( V[i], V[i+1], P) > 0)  // P left of  edge
                     wn++;            // have  a valid up intersect
            }
        }
        else {                        // start y > P.y (no test needed)
            if (V[i+1].y  <= P.y){     // a downward crossing
                 if (isLeft( V[i], V[i+1], P) < 0)  // P right of  edge
                     wn--;            // have  a valid down intersect
            }
        }
    }
    return wn;
}
//===================================================================
