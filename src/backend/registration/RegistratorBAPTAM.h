#ifndef _CSLAM_REG_BAPTAM_H_
#define _CSLAM_REG_BAPTAM_H_

#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/smart_ptr.hpp>

#include <sophus/se3.hpp>

#include <ptam/Bundle.h>

#include <ptam/PolynomialCamera.h>
#include <cs_geometry/Camera.h>

#include "Registrator.h"

namespace backend {

class RegistratorBAPTAM : public Registrator {
public:
    RegistratorBAPTAM(const cs_geom::Camera& cam,
                  double rob = 3.0,
                  double thresh2D = 2.0,
                  double threshD = 0.2);

    virtual void solve(const std::vector<cv::KeyPoint>& kpts0,
               const std::vector<float>& depths0,
               const std::vector<cv::KeyPoint>& kpts1,
               const std::vector<float>& depths1,
               const std::vector<cv::DMatch>& matches,
               const Sophus::SE3d& cam0T1prior = Sophus::SE3d());

protected:
    void reset();

    boost::shared_ptr<ptam::PolynomialCamera> ptam_cam_;
    boost::scoped_ptr<ptam::Bundle> bundler_;
};

}

#endif /* _CSLAM_REG_BAPTAM_H_ */
