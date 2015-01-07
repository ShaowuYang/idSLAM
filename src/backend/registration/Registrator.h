#ifndef _CSLAM_BA_PROBLEM_H_
#define _CSLAM_BA_PROBLEM_H_

#include <sophus/se3.hpp>

#include <cs_geometry/Camera.h>

namespace cslam {

class Registrator {
public:
    Registrator(const cs_geom::Camera& cam, double rob = 3.0,
              double thresh2D = 5.0,
              double threshD = 0.3) : cam_(cam), rob_(rob), thresh2D_(thresh2D), threshD_(threshD), nMatches_(0), nInliers_(0), nInliersDepth_(0) {}

    virtual void solve(const std::vector<cv::KeyPoint>& kpts0,
                  const std::vector<float>& depths0,
                  const std::vector<cv::KeyPoint>& kpts1,
                  const std::vector<float>& depths1,
                  const std::vector<cv::DMatch>& matches,
                  const Sophus::SE3d& cam0T1prior) = 0;

    Sophus::SE3d relPose1T0() { return relPose0T1_.inverse(); }
    Sophus::SE3d relPose0T1() { return relPose0T1_; }

    int nMatches() { return nMatches_; }
    int nInliers() { return nInliers_; }
    int nInliersDepth() { return nInliersDepth_; }

    std::vector<cv::DMatch> inliers() { return inliers_; }
protected:
    const cs_geom::Camera& cam_;
    double rob_, thresh2D_, threshD_;

    Sophus::SE3d relPose0T1_;

    int nMatches_, nInliers_, nInliersDepth_;

    std::vector<cv::DMatch> inliers_;
};

}

#endif /* _CSLAM_BA_PROBLEM_H_ */
