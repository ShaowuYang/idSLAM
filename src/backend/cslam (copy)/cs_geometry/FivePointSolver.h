/*
 * FivePointSolver.h
 *
 *  Created on: Apr 22, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#ifndef _FIVEPOINTSOLVER_H_
#define _FIVEPOINTSOLVER_H_

#include <vector>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace cs_geom {

class FivePointHypothesis {
public:
    FivePointHypothesis() : score(0.0) { }

    FivePointHypothesis(const std::vector<int>& nindizes, const Eigen::Matrix3d& nE) :
        score(0.0),
        E(nE)
    {
        for (int i = 0; i < 5; i++)
            indizes[i] = nindizes[i];
    }

    FivePointHypothesis(const FivePointHypothesis& o) {
        score = o.score;
        E = o.E;
        memcpy(indizes,o.indizes,sizeof(indizes));
    }

    static bool compare(const FivePointHypothesis& h1, const FivePointHypothesis& h2) { return h1.score > h2.score; }

    void get2RT1(const std::vector<cv::Point2d>& points1,
               const std::vector<cv::Point2d>& points2,
               Eigen::Matrix3d& R,
               Eigen::Vector3d& T);

    inline double scorePair(const cv::Point2d& q1, const cv::Point2d& q2) {
        return pointLineError2(q1, q2) < 0.005*0.005;
    }

    double reprojError2(const cv::Point2d& q1, const cv::Point2d& q2);
    double pointLineError2(const cv::Point2d& q1, const cv::Point2d& q2);
    double sampsonError2(const cv::Point2d& q1, const cv::Point2d& q2);

    double score;
    Eigen::Matrix3d E;
    int indizes[5];
};

class FivePointSolver {
public:
    static FivePointHypothesis findEPreemptiveRANSAC(const std::vector<cv::Point2d>& points1,
                                                     const std::vector<cv::Point2d>& points2,
                                                     int nMaxObservations,
                                                     int nInitialHypotheses,
                                                     int blockSize);

    static void triangulate(const std::vector<cv::Point2d>& q1,
                            const std::vector<cv::Point2d>& q2,
                            const Eigen::Matrix3d& R,
                            const Eigen::Vector3d& T,
                            std::vector<Eigen::Vector3d>& X);

    static void fivePointNisterGroebner(const std::vector<cv::Point2d>& q1, const std::vector<cv::Point2d>& q2, std::vector<Eigen::Matrix3d>& E);
protected:

    static inline int preemption(int i, int M, int B) { return M*pow(2, - i/B); }
    typedef std::pair<int,int> Match;
};


} // namespace


#endif /* _FIVEPOINTSOLVER_H_ */
