/*
  All polygon related code should be located into polygon class.
  mainly two parts:
  1. check the type of a polygon, a convex polygon or others
  2. check whether a point locates in a polygon or outside.
  */
#ifndef POLYGON_H
#define POLYGON_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>

namespace ptam{
/* Program to Classify a Polygon's Shape */

/* CheckTriple tests three consecutive points for change of direction
 * and for orientation.
 */
#define CheckTriple							\
    if ( (thisDir = Compare(second, third)) == -curDir )		\
        ++dirChanges;						\
    curDir = thisDir;						\
    if ( thisSign = WhichSide(first, second, third) ) {		\
        if ( angleSign == -thisSign )				\
        return NotConvex;					\
        angleSign = thisSign;					\
    }								\
    first = second; second = third;

// CW: clockwise
// CCW: counter-clockwise. Only those two classes are what we want in landing pad tracking
typedef enum { NotConvex, NotConvexDegenerate,
           ConvexDegenerate, ConvexCCW, ConvexCW } PolygonClass;

inline int WhichSide(cv::Point p, cv::Point q, cv::Point r)		/* Given a directed line pq, determine	*/
 /* whether qr turns CW or CCW.		*/
{
    double result;
    result = (p.x - q.x) * (q.y - r.y) - (p.y - q.y) * (q.x - r.x);
    if (result < 0) return -1;	/* q lies to the left  (qr turns CW).	*/
    if (result > 0) return  1;	/* q lies to the right (qr turns CCW).	*/
    return 0;			/* q lies on the line from p to r.	*/
}

inline int Compare(cv::Point p, cv::Point q)		/* Lexicographic comparison of p and q	*/
{
    // for landing pad detection, a threshold of 20 pixels is set
    if (p.x < q.x-20) return -1;	/* p is less than q.			*/
    if (p.x > q.x+20) return  1;	/* p is greater than q.			*/
    if (p.y < q.y-20) return -1;	/* p is less than q.			*/
    if (p.y > q.y+20) return  1;	/* p is greater than q.			*/
    return 0;			/* p is equal to q.			*/
}

inline int GetPoint(std::vector<cv::Point> f, int &i, cv::Point *p)		/* Read p's x- and y-coordinate from the ith element of f	*/
 /* and return true, iff successful.	*/
{
    if (i >= 0 && i <= f.size()){
        p->x = f[i].x;
        p->y = f[i].y;
        i ++;
        return true;
    }
    else
        return false;
//    return !feof(f) && (2 == fscanf(f, "%lf%lf", &(p->x), &(p->y)));
}

inline int GetDifferentPoint(std::vector<cv::Point> f, int &i, cv::Point previous, cv::Point *next)
 /* Read next point into 'next' until it */
 /* is different from 'previous' and	*/
{				/* return true iff successful.		*/
    int eof;
    while((i < f.size()) && (eof = GetPoint(f, i, next)) && (Compare(previous, *next) == 0));
    return eof;
}

/* Program to check a point inside/outside a Polygon */
// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"
inline int isLeft( cv::Point P0, cv::Point P1, cv::Point P2 )
{
    return ( (P1.x - P0.x) * (P2.y - P0.y)
            - (P2.x -  P0.x) * (P1.y - P0.y) );
}

////////////////////////////////////////
// main functions for polygon
namespace Polygon{
    PolygonClass ClassifyPolygon(std::vector<cv::Point> f, int n);

    int cn_PnPoly( cv::Point P, std::vector<cv::Point> V, int n );
    int wn_PnPoly( cv::Point P, std::vector<cv::Point> V, int n );

    // find the maximum subimage corners from a set of points
    inline bool findmaxsubimg(std::vector<cv::Point> points, cv::Size size, cv::Point &lefttop, cv::Point &rightbottom)
    {
        lefttop = cv::Point(10000, 10000);
        rightbottom = cv::Point(0, 0);
        for (int i = 0; i < points.size(); i ++){
            if (points[i].x < lefttop.x)
                lefttop.x = points[i].x;
            if (points[i].y < lefttop.y)
                lefttop.y = points[i].y;
            if (points[i].x > rightbottom.x)
                rightbottom.x = points[i].x;
            if (points[i].y > rightbottom.y)
                rightbottom.y = points[i].y;
        }
        // subimage need to be inside the image
        if (lefttop.x < 0)
            lefttop.x = 0;
        else if (lefttop.x > size.width)
            return false;
        if (lefttop.y < 0)
            lefttop.y = 0;
        else if (lefttop.y > size.height)
            return false;
        if (rightbottom.x > size.width)
            rightbottom.x = size.width;
        else if (rightbottom.x < 0)
            return false;
        if (rightbottom.y > size.height)
            rightbottom.y = size.height;
        else if (rightbottom.y < 0)
            return false;

        return true;
    }

    inline cv::Point mass_center(std::vector<cv::Point> f)
    {
        cv::Point center(0, 0);
        for (int i = 0; i < f.size(); i ++)
        {
            center.x += f[i].x;
            center.y += f[i].y;
        }
        center.x = center.x/f.size();
        center.y = center.y/f.size();

        return center;
    }
    inline double square(double element){ return element*element; }
    // @ output: whether distances of the mass center of the polygon to its corners
    //           is evenly distributed. Simply check whether this polygon could be the object we need to detect
    inline bool masscenter_distribute_normal(cv::Point center, std::vector<cv::Point> f)
    {
        if (f.size() < 4)
            return false;
        double mindis = 1000, maxdis = 0, averdis = 0;
        double fdis[f.size()];
        for (int i = 0; i < f.size(); i ++)
        {
            double dis = sqrt(square(f[i].x-center.x) + square(f[i].y-center.y));
            fdis[i] = dis;
            averdis +=dis;
            if (dis < mindis)
                mindis = dis;
            if (dis > maxdis)
                maxdis = dis;
        }
        averdis = averdis / f.size();

        // by checking whether there is a very distinguish corner, e.g.
        //        --------------
        //        |          /
        //        |      /
        //        -----
        if (mindis < maxdis/2.0 || (maxdis - averdis) > mindis)
            return false;

        // and check whether there are at least two continuous slightly distinguish corners, e.g.
        // ----------------
        //   \          /
        //      \    /
        //        --
        // maybe both clock / counterclockwise should be checked
        int continug = 0, continus = 0;
        for (int i = 0; i < f.size(); i ++)
        {
            if (fdis[i] > mindis*1.5)
            {
                continug ++;
                if (continug > 1)
                    return false;
            }
            else
                continug = 0;
            if (fdis[i] < maxdis/1.5)
            {
                continus ++;
                if (continus > 1)
                    return false;
            }
            else
                continus = 0;
        }

        // also, such case should be eliminated
        // -------------------------
        // |                        |
        // -------------------------

        return true;
    }
    // shrink a quadrilateral. corners need to be with size 4
    inline std::vector<cv::Point> shrinkpolygon(std::vector<cv::Point> corners, double factor){
        std::vector<cv::Point> cornersout;
        assert(corners.size() == 4);
        // shrink corners 0-2
        cv::Point center;
        center.x = corners[0].x + corners[2].x;
        center.y = corners[0].y + corners[2].y;
        corners[0].x += (center.x -corners[0].x)*(1-factor);
        corners[0].y += (center.y -corners[0].y)*(1-factor);
        corners[2].x += (center.x-corners[2].x)*(1-factor);
        corners[2].y += (center.y-corners[2].y)*(1-factor);
        // shrink corners 1-3
        center.x = corners[1].x + corners[3].x;
        center.y = corners[1].y + corners[3].y;
        corners[1].x += (center.x-corners[1].x)*(1-factor);
        corners[1].y += (center.y-corners[1].y)*(1-factor);
        corners[3].x += (center.x-corners[3].x)*(1-factor);
        corners[3].y += (center.y-corners[3].y)*(1-factor);

        cornersout = corners;
        return cornersout;
    }
}
}
#endif // POLYGON_H
