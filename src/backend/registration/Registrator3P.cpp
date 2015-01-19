#include "Registrator3P.h"

#include <opencv2/core/eigen.hpp>
#include <cs_geometry/Math.h>
#include <cs_geometry/Conversions.h>
#include <ptam/MapPoint.h>

using namespace backend;
using namespace cs_geom;


#define N_SAMPLES 4

Sophus::SE3d Registrator3P::solve(const ptam::KeyFrame& kfa, const ptam::KeyFrame& kfb,
                                  const std::vector<cv::DMatch>& matches)
{
    if (matches.size() < 5)
        return Sophus::SE3d();

    // Generate nHyp_ hypotheses:
    std::vector<Hypothesis3P> hyp;
    while (hyp.size() < nHyp_) {
        std::vector<int> sampleInd;

        // choose x random matches
        randSampleNoReplacement(matches.size(), N_SAMPLES, sampleInd);

        std::vector<cv::Point3f> mapPointsA(N_SAMPLES);
        std::vector<cv::Point2f> imagePointsB(N_SAMPLES);

        for (int i = 0; i < N_SAMPLES; i++) {
            const int& ind = sampleInd[i];
            const Eigen::Vector3d& mpa = cs_geom::toEigenVec(kfa.mapPoints[matches[ind].queryIdx]->v3RelativePos);
            mapPointsA[i]   = cv::Point3f(mpa[0], mpa[1], mpa[2]);
            imagePointsB[i] = kfb.keypoints[matches[ind].trainIdx].pt;
        }

        cv::Mat rvec(3,1,cv::DataType<double>::type);
        cv::Mat tvec(3,1,cv::DataType<double>::type);
        /// here we need the model of camera B!
        int camNum = kfb.nSourceCamera;
        if (cv::solvePnP(mapPointsA, imagePointsB, cam_[camNum].K(), cam_[camNum].D(), rvec, tvec,  false, cv::EPNP)) {
            Sophus::SE3d se3;
            Eigen::Vector3d r;
            cv::cv2eigen(rvec, r);
            cv::cv2eigen(tvec, se3.translation());
            se3.so3() = Sophus::SO3d::exp(r);
            hyp.push_back(Hypothesis3P(se3));
        }
    }

    // Preemptive scoring
    // determine the order in which observations will be evaluated
    std::vector<int> oInd;
    randPerm(matches.size(),oInd);

    // start preemptive scoring
    int i = 0;
    int pr = preemption(i, nHyp_, blockSize_);
    while (i < nMaxObs_ && i < (int) matches.size() && pr > 1) {
        // observation oInd(i) consists of one pair of points:

        const cv::DMatch& match = matches[oInd[i]];
        const Eigen::Vector3d& mpa   = cs_geom::toEigenVec(kfa.mapPoints[match.queryIdx]->v3RelativePos);
        const cv::Point2f&     imbcv = kfb.keypoints[match.trainIdx].pt;
        Eigen::Vector2d imb(imbcv.x, imbcv.y);

        // update score for all hypotheses w.r.t. observation oInd(i)
        for (int h = 0; h < (int) hyp.size(); h++) {
            Eigen::Vector2d err = cam_[kfb.nSourceCamera].project3DtoPixel(hyp[h].relPose*mpa) - imb;
            hyp[h].score -= log(1.0 + err.dot(err));
//            hyp[h].score += err.dot(err) < 3.0;
        }

        i++;
        int prnext = preemption(i, nHyp_, blockSize_);
        if (prnext != pr) {
            // select best hypotheses
            std::nth_element(hyp.begin(), hyp.begin() + prnext, hyp.end(), Hypothesis3P::compare);
            // now the first prnext elements of h contain the best hypotheses, erase the rest
            hyp.erase(hyp.begin() + prnext, hyp.end());
        }
        pr = prnext;
    }
    // preemptive scoring is done

    // select the single best hypothesis of possibly more than one remaining
    std::nth_element(hyp.begin(),hyp.begin() + 1, hyp.end(), Hypothesis3P::compare);

//    std::cout << "preemptive scoring using " << i << " observations done." <<  std::endl;
//    std::cout << hyp[0].relPose.inverse().matrix();
    return hyp[0].relPose.inverse();
}

std::vector<cv::DMatch> Registrator3P::getInliers(const ptam::KeyFrame& kfa, const ptam::KeyFrame& kfb,
                                                  const std::vector<cv::DMatch>& matches,
                                                  const Sophus::SE3d& relPoseAB,
                                                  double threshold,
                                                  std::vector<Observation>& obs)
{
    Sophus::SE3d relPoseBA = relPoseAB.inverse();
    std::vector<cv::DMatch> inliers;
    double thresh2 = threshold*threshold;
    for (uint i = 0; i < matches.size(); i++) {
        const cv::DMatch& m = matches[i];

        const Eigen::Vector3d& mpa   = cs_geom::toEigenVec(kfa.mapPoints[m.queryIdx]->v3RelativePos);
        const cv::Point2f&     imbcv = kfb.keypoints[m.trainIdx].pt;
        Eigen::Vector2d ipb(imbcv.x, imbcv.y);

        Eigen::Vector2d err = cam_[kfb.nSourceCamera].project3DtoPixel(relPoseBA*mpa) - ipb;;

        if (err.dot(err) < thresh2) {
            inliers.push_back(m);
            obs.push_back(Observation(mpa, cam_[kfb.nSourceCamera].unprojectPixel(ipb)));
        }
    }

    return inliers;
}

bool Registrator3P::solvePnP_RANSAC(const ptam::KeyFrame& kfa, const ptam::KeyFrame& kfb,
                                  const std::vector<cv::DMatch>& matches, Sophus::SE3d &result, int minInliers)
{
    if (matches.size() < 5)
        return false;

    std::vector<cv::Point3f> mapPointsA(matches.size());
    std::vector<cv::Point2f> imagePointsB(matches.size());

    for (int i = 0; i < matches.size(); i++) {
        const int& ind = i;
        const Eigen::Vector3d& mpa = cs_geom::toEigenVec(kfa.mapPoints[matches[ind].queryIdx]->v3RelativePos);
        mapPointsA[i]   = cv::Point3f(mpa[0], mpa[1], mpa[2]);
        imagePointsB[i] = kfb.keypoints[matches[ind].trainIdx].pt;
    }

    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    std::vector<int> inliers;
    /// here we need the model of camera B!
    int camNum = kfb.nSourceCamera;
    cv::solvePnPRansac(mapPointsA, imagePointsB, cam_[camNum].K(), cam_[camNum].D(), rvec, tvec,  false, 300, 6.0, minInliers*5, inliers, cv::EPNP);
    if (inliers.size() > minInliers) {
        Eigen::Vector3d r;
        Sophus::SE3 se3; // TBA
        cv::cv2eigen(rvec, r);
        cv::cv2eigen(tvec, se3.translation());
        se3.so3() = Sophus::SO3d::exp(r);
        result = se3.inverse(); // TAB
        return true;
    }
    return false;
}
