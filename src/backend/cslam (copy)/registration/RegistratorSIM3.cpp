#include "RegistratorSIM3.h"

#include <TooN/sim3.h>

#include <ptam/MEstimator.h>

#include <fstream>

using namespace cslam;

using namespace std;

void RegistratorSIM3::solve(const std::vector<Observation>& obsAB,
                                     const std::vector<Observation>& obsBA)
{
    double lastErr2 = std::numeric_limits<double>::max();

    vector<double> error2(obsAB.size() + obsBA.size());
    double sigma2;

    for (int i = 0; i < 20; i++) {
        // Compute error statistics required for robust reweighting kernels
        TooN::Vector<2> err;
        // inverse measurements
        for (uint i = 0; i < obsAB.size(); i++) {
            calcErrAB(obsAB[i], err);
            error2[i] = err*err;
        }

        // forward measurements
        for (uint i = 0; i < obsBA.size(); i++) {
            calcErrBA(obsBA[i], err);
            error2[i + obsAB.size()] = err*err;
        }

        sigma2 = ptam::Huber::FindSigmaSquared(error2);

        wls_.clear();
        wls_.add_prior(0.01);

        double err2 = 0.0;
        TooN::Matrix<2,7> jac;

        // inverse measurements
        for (uint i = 0; i < obsAB.size(); i++) {
            calcErrJacobianAB(obsAB[i], err, jac);
            double currErr2 = err*err;
            double weight = ptam::Huber::Weight(currErr2, sigma2);
            wls_.add_mJ(err[0], jac[0], weight);
            wls_.add_mJ(err[1], jac[1], weight);
            err2 += currErr2;
        }

        // forward measurements
        for (uint i = 0; i < obsBA.size(); i++) {
            calcErrJacobianBA(obsBA[i], err, jac);
            double currErr2 = err*err;
            double weight = ptam::Huber::Weight(currErr2, sigma2);
            wls_.add_mJ(err[0], jac[0], weight);
            wls_.add_mJ(err[1], jac[1], weight);
            err2 += currErr2;
        }

        wls_.compute();
        TooN::Vector<7> mu = wls_.get_mu();

        if (!useSIM3_)
            mu[6] = 0.0;

        estATB_ = TooN::SIM3<>::exp(mu) * estATB_;
        estBTA_ = estATB_.inverse();
        lastErr2 = err2;
    }
}

Sophus::Sim3d RegistratorSIM3::aTb_sim3()
{
    TooN::Vector<7> log = estATB_.ln();

    Sophus::Sim3d::Tangent log_sophus;
    for (int i = 0; i < 7; i++)
        log_sophus[i] = log[i];

    return Sophus::Sim3d::exp(log_sophus);
}

Sophus::Sim3d RegistratorSIM3::bTa_sim3()
{
    TooN::Vector<7> log = estBTA_.ln();

    Sophus::Sim3d::Tangent log_sophus;
    for (int i = 0; i < 7; i++)
        log_sophus[i] = log[i];

    return Sophus::Sim3d::exp(log_sophus);
}

Sophus::SE3d RegistratorSIM3::aTb_se3()
{
    TooN::Vector<7> log = estATB_.ln();

    Sophus::SE3d::Tangent log_sophus;
    for (int i = 0; i < 6; i++)
        log_sophus[i] = log[i];

    return Sophus::SE3d::exp(log_sophus);
}

Sophus::SE3d RegistratorSIM3::bTa_se3()
{
    TooN::Vector<7> log = estBTA_.ln();

    Sophus::SE3d::Tangent log_sophus;
    for (int i = 0; i < 6; i++)
        log_sophus[i] = log[i];

    return Sophus::SE3d::exp(log_sophus);
}

void RegistratorSIM3::calcErrAB(const Observation& obsAB,
                                TooN::Vector<2>& err)
{
    // require inverse pose - point composition
    const TooN::Vector<3>& p = obsAB.mapPoint;
    TooN::Vector<3> q = estBTA_*p;
    err = obsAB.imagePoint - TooN::project(q);
}

void RegistratorSIM3::calcErrJacobianAB(const Observation& obsAB,
                                        TooN::Vector<2>& err,
                                        TooN::Matrix<2,7>& jac)
{
    // require inverse pose - point composition
    const TooN::Vector<3>& p = obsAB.mapPoint;
    TooN::Vector<3> q = estBTA_*p;

    err = obsAB.imagePoint - TooN::project(q);

    double qzinv = 1.0/q[2];

    TooN::Matrix<3,7> Jpose(TooN::Data(
                                -1.0, 0.0, 0.0, 0.0, -p[2], p[1], -p[0],
                                0.0, -1.0, 0.0, p[2], 0.0, -p[0], -p[1],
                                0.0, 0.0, -1.0, -p[1], p[0], 0.0, -p[2]));

    TooN::Matrix<2,3> Jproj(TooN::Data(
                                qzinv, 0.0, -q[0]*qzinv*qzinv,
                                0.0, qzinv, -q[1]*qzinv*qzinv));

    double scale = estBTA_.get_scale();
    jac = Jproj*estBTA_.get_rotation()*Jpose/(scale*scale);
}

void RegistratorSIM3::calcErrBA(const Observation& obsBA,
                                TooN::Vector<2>& err)
{
    // forward projection
    const TooN::Vector<3>& p = obsBA.mapPoint;
    TooN::Vector<3> q = estATB_*p;
    err = obsBA.imagePoint - TooN::project(q);
}

void RegistratorSIM3::calcErrJacobianBA(const Observation& obsBA,
                                        TooN::Vector<2>& err,
                                        TooN::Matrix<2,7>& jac)
{
    // forward projection
    const TooN::Vector<3>& p = obsBA.mapPoint;
    TooN::Vector<3> q = estATB_*p;

    err = obsBA.imagePoint - TooN::project(q);

    double qzinv = 1.0/q[2];
    double qzinv2 = qzinv*qzinv;

    TooN::Matrix<3,7> Jpose(TooN::Data(
                                1.0, 0.0, 0.0, 0.0, q[2], -q[1], q[0],
                                0.0, 1.0, 0.0, -q[2], 0.0, q[0], q[1],
                                0.0, 0.0, 1.0, q[1], -q[0], 0.0, q[2]));

    TooN::Matrix<2,3> Jproj(TooN::Data(
                                qzinv, 0.0, -q[0]*qzinv*qzinv,
                                0.0, qzinv, -q[1]*qzinv*qzinv));

    jac = Jproj*Jpose;
}
