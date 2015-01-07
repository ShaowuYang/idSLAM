#ifndef _REGISTRATOR_3P_H_
#define _REGISTRATOR_3P_H_

#include <TooN/TooN.h>

#include <cs_geometry/Camera.h>
#include <ptam/KeyFrame.h>

namespace cslam {

struct Hypothesis3P {
    Hypothesis3P() : score(0.0) { }
    Hypothesis3P(const Sophus::SE3d& relPose) : relPose(relPose), score(0.0) { }

    static bool compare(const Hypothesis3P& h1, const Hypothesis3P& h2) {
        return h1.score > h2.score;
    }

    Sophus::SE3d relPose;
    double score;
};

struct Observation {
    Observation(const Eigen::Vector3d& mp, const Eigen::Vector2d& ip) {
        mapPoint[0] = mp[0]; mapPoint[1] = mp[1]; mapPoint[2] = mp[2];
        imagePoint[0] = ip[0]; imagePoint[1] = ip[1];
    }

    TooN::Vector<3> mapPoint;
    TooN::Vector<2> imagePoint;
};

class Registrator3P {
public:
    Registrator3P(const cs_geom::Camera& cam, int nMaxObs = 500, int nHyp = 400, int blockSize = 50) :
        cam_(cam), nMaxObs_(nMaxObs), nHyp_(nHyp), blockSize_(blockSize) {

    }

    // Try to find kfa's map points to kfb's corners. Return: ^A T_B
    Sophus::SE3d solve(const ptam::KeyFrame& kfa, const ptam::KeyFrame& kfb, const std::vector<cv::DMatch>& matches);
    std::vector<cv::DMatch> getInliers(const ptam::KeyFrame& kfa, const ptam::KeyFrame& kfb,
                                       const std::vector<cv::DMatch>& matches,
                                       const Sophus::SE3d& relPoseBA,
                                       double threshold,
                                       std::vector<Observation>& obs);

    protected:
    static inline int preemption(int i, int M, int B) { return M*pow(2, - i/B); }

    const cs_geom::Camera& cam_;
    int nMaxObs_, nHyp_, blockSize_;
};

}

#endif /* _REGISTRATOR_3P_H_ */
