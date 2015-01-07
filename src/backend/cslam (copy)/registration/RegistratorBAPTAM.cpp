#include "RegistratorBAPTAM.h"

#include <fenv.h>

#include <ptam/PolynomialCamera.h>
#include <ptam/Bundle.h>

using namespace cslam;
using namespace ptam;
using namespace std;

Sophus::SE3d toon2sophusSE3(const TooN::SE3<>& toon) {
    TooN::Vector<6> tlog = toon.ln();
    Sophus::SE3d::Tangent slog;
    for (int i = 0; i < 6; i++)
        slog[i] = tlog[i];

    return Sophus::SE3d::exp(slog);
}

TooN::SE3<> sophus3toonSE3(const Sophus::SE3d& sophus) {
    Sophus::SE3d::Tangent slog = sophus.log();
    TooN::Vector<6> tlog;
    for (int i = 0; i < 6; i++)
        tlog[i] = slog[i];

    return TooN::SE3<>(tlog);
}

RegistratorBAPTAM::RegistratorBAPTAM(const cs_geom::Camera& cam,
                             double rob, double thresh2D, double threshD) : Registrator(cam)
{
//    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
    cv::Size size(cam.width(), cam.height());

    // Ownership of pointer to ptam_cam is transferred to bundler:
//    ptam_cam_.reset(new ptam::PolynomialCamera(cam.K(), cam.D(), size));

}

void RegistratorBAPTAM::solve(const std::vector<cv::KeyPoint>& kpts0,
                     const std::vector<float>& depths0,
                     const std::vector<cv::KeyPoint>& kpts1,
                     const std::vector<float>& depths1,
                     const std::vector<cv::DMatch>& matches,
                     const Sophus::SE3d& cam0T1prior)
{
    reset();

    nMatches_ = matches.size();

    bundler_->AddCamera(TooN::SE3<>(), true);
    bundler_->AddCamera(sophus3toonSE3(cam0T1prior.inverse()), false);

    const double dErrorScale = 6.331e-3;

    for (uint i = 0; i < matches.size(); i++) {
        int ind1 = matches[i].trainIdx;
        int ind0 = matches[i].queryIdx;
        Eigen::Vector2d u0(kpts0[ind0].pt.x, kpts0[ind0].pt.y);
        Eigen::Vector2d u1(kpts1[ind1].pt.x, kpts1[ind1].pt.y);

        // Initialize pointParams:
        double dIinit = depths0[ind0] > 0 ? depths0[ind0] : depths1[ind1] > 0 ? depths1[ind1] : 2.0;
        Eigen::Vector3d pIinit = dIinit*cam_.unprojectPixelTo3D(u0);

        TooN::Vector<3> v3pos;
        v3pos[0] = pIinit[0]; v3pos[1] = pIinit[1]; v3pos[2] = pIinit[2];
        int id = bundler_->AddPoint(v3pos);

        TooN::Vector<2> v2u0; v2u0[0] = u0[0]; v2u0[1] = u0[1];
        TooN::Vector<2> v2u1; v2u1[0] = u1[0]; v2u1[1] = u1[1];
        bundler_->AddMeas(0, id, v2u0, 1.0);
        bundler_->AddMeas(1, id, v2u1, 1.0);

        if (depths0[ind0] > 0) {
            bundler_->AddMeas(0, id, depths0[ind0], depths0[ind0]*depths0[ind0]*dErrorScale);
        }

        if (depths1[ind1] > 0) {
            bundler_->AddMeas(1, id, depths1[ind1], depths1[ind1]*depths1[ind1]*dErrorScale);
        }
    }

    bool abort = false;
    bundler_->Compute(&abort);

    relPose0T1_ = toon2sophusSE3(bundler_->GetCamera(1).inverse()); // bundler returns: 1TW

    Sophus::SE3d relPose1T0 = relPose0T1_.inverse();

    // determine inliers
    inliers_.clear();

    double thresh2D_2 = thresh2D_*thresh2D_;
    for (uint i = 0; i < matches.size(); i++) {
        int ind1 = matches[i].trainIdx;
        int ind0 = matches[i].queryIdx;
        Eigen::Vector2d u0(kpts0[ind0].pt.x, kpts0[ind0].pt.y);
        Eigen::Vector2d u1(kpts1[ind1].pt.x, kpts1[ind1].pt.y);

        TooN::Vector<3> pp = bundler_->GetPoint(i);
        Eigen::Vector3d p0est(pp[0], pp[1], pp[2]);
        Eigen::Vector3d p1est = relPose1T0*p0est;
        Eigen::Vector2d u0est = cam_.project3DtoPixel(p0est);
        Eigen::Vector2d u1est = cam_.project3DtoPixel(p1est);

        if ((u0 - u0est).squaredNorm() < thresh2D_2 &&
                (u1 - u1est).squaredNorm() < thresh2D_2) {
            // not a 2d outlier:

            if ((depths0[ind0] > 0 && fabs(p0est[2] - depths0[ind0]) < threshD_)
                    || (depths1[ind1] > 0 && fabs(p1est[2] - depths1[ind1]) < threshD_)) {
                nInliersDepth_++;
            }
            nInliers_++;
            inliers_.push_back(matches[i]);
        } // end if not a 2d outlier
    } // end for each match
}

void RegistratorBAPTAM::reset()
{
//    bundler_.reset(new ptam::Bundle(ptam_cam_, 40, 1e-8, 0, "Tukey", 50));
    nMatches_ = nInliers_ = nInliersDepth_ = 0;
    inliers_.clear();
}
