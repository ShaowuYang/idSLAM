#include "Registrator5P.h"

#include <cs_geometry/FivePointSolver.h>

#include <opencv2/calib3d/calib3d.hpp>

using namespace cs_geom;
using namespace cslam;
using namespace std;

Registrator5P::Registrator5P(const cs_geom::Camera& cam,
                             double rob, double thresh2D, double threshD) : Registrator(cam)
{

}

void Registrator5P::solve(const std::vector<cv::KeyPoint>& kpts0,
                     const std::vector<float>& depths0,
                     const std::vector<cv::KeyPoint>& kpts1,
                     const std::vector<float>& depths1,
                     const std::vector<cv::DMatch>& matches,
                     const Sophus::SE3d&)
{
    reset();
    nMatches_ = matches.size();

    // Unproject points:
    std::vector<cv::Point2d> q0;
    std::vector<cv::Point2d> q1;
    for (unsigned int i = 0; i < matches.size(); i++) {
        int i1 = matches[i].trainIdx;
        int i0 = matches[i].queryIdx;

        assert(i0 < kpts0.size());
        assert(i1 < kpts1.size());

        q0.push_back(cam_.unprojectPixel(cv::Point2d(kpts0[i0].pt.x,
                                                     kpts0[i0].pt.y)));
        q1.push_back(cam_.unprojectPixel(cv::Point2d(kpts1[i1].pt.x,
                                                     kpts1[i1].pt.y)));
    }

    // Determine Essential matrix:
    FivePointHypothesis h = FivePointSolver::findEPreemptiveRANSAC(q0, q1, 2000, 2000, 400);
    std::cout << "score: " << h.score << std::endl;
    std::cout << "best E : " << h.E << std::endl;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    h.get2RT1(q0, q1, R, T);

    Sophus::SE3d relPose1T0;
    relPose1T0.so3() = R;
    relPose1T0.translation() = T;

    relPose0T1_ = relPose1T0.inverse();

    std::cout << "raw rel pose : " << endl << relPose0T1_.matrix3x4() << std::endl;
    // Determine scale factor from depth measurements
    std::vector<Eigen::Vector3d> X;
    std::vector<double> scaleFactors;
    FivePointSolver::triangulate(q0, q1, R, T, X);
    for (unsigned int i = 0; i < matches.size(); i++) {
        int i1 = matches[i].trainIdx;
        int i0 = matches[i].queryIdx;
        assert(i0 < kpts0.size());
        assert(i1 < kpts1.size());

        if (X[i][2] <= 0.0)
            continue;

        if (depths0[i0] > 0.0) {
            scaleFactors.push_back(depths0[i0]/X[i][2]);
        }

        if (depths1[i1] > 0.0) {
            Eigen::Vector3d Xt = relPose0T1_.inverse()*X[i];
            if (Xt[2] <= 0.0)
                continue;
            scaleFactors.push_back(depths1[i1]/Xt[2]);
        }
    }

    // Median scale factor:
    size_t n = scaleFactors.size() / 2;
    nth_element(scaleFactors.begin(), scaleFactors.begin() + n, scaleFactors.end());
    double scaleFactor  = scaleFactors[n];
    std::cout << "scalefactor: " << scaleFactor << std::endl;

    relPose0T1_.translation() *= scaleFactor;

    // determine inliers
    inliers_.clear();

    double thresh2D_2 = thresh2D_*thresh2D_;
    for (uint i = 0; i < matches.size(); i++) {
        int i1 = matches[i].trainIdx;
        int i0 = matches[i].queryIdx;
        assert(i0 < kpts0.size());
        assert(i1 < kpts1.size());

        double err = h.pointLineError2(cam_.unprojectPixel(cv::Point2d(kpts0[i0].pt.x,
                                                                    kpts0[i0].pt.y)),
                                       cam_.unprojectPixel(cv::Point2d(kpts1[i1].pt.x,
                                                                    kpts1[i1].pt.y)));
        if (err < thresh2D_2) {
            nInliers_++;
            inliers_.push_back(matches[i]);
        }

//        Eigen::Vector2d u0(kpts0[i0].pt.x, kpts0[i0].pt.y);
//        Eigen::Vector2d u1(kpts1[i1].pt.x, kpts1[i1].pt.y);

//        Eigen::Vector3d p0est = X[i]*scaleFactor;
//        Eigen::Vector3d p1est = relPose1T0*p0est;
//        Eigen::Vector2d u0est = cam_.project3DtoPixel(p0est);
//        Eigen::Vector2d u1est = cam_.project3DtoPixel(p1est);

//        if ((u0 - u0est).squaredNorm() < thresh2D_2 &&
//                (u1 - u1est).squaredNorm() < thresh2D_2) {
//            // not a 2d outlier:

//            if ((depths0[i0] > 0 && fabs(p0est[2] - depths0[i0]) < threshD_)
//                    || (depths1[i1] > 0 && fabs(p1est[2] - depths1[i1]) < threshD_)) {
//                nInliersDepth_++;
//            }
//            nInliers_++;
//            inliers_.push_back(matches[i]);
//        } // end if not a 2d outlier
    } // end for each match

//    cout << "RANSAC: inliers: " << inliers().size() << endl;
}

void Registrator5P::reset()
{
    nMatches_ = nInliers_ = nInliersDepth_ = 0;
    inliers_.clear();
}
