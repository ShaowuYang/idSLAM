#include "RegistratorKFs.h"
using namespace std;

namespace backend {

RegistratorKFs::RegistratorKFs(const cs_geom::Camera *cam,
                               double nMinInliers, double threshPx, double maxErrAngle,
                               bool useSIM3, double maxerrdis)
    : nMinInliers_(nMinInliers), threshPx_(threshPx), maxErrAngle_(maxErrAngle), maxErrDis_(maxerrdis)
{
    matcher_.reset(new cv::BFMatcher(cv::NORM_HAMMING, true));
    reg_3p_.reset(new Registrator3P(cam, 1+AddCamNumber));
    reg_sim3_.reset(new RegistratorSIM3(useSIM3));
}

boost::shared_ptr<ptam::Edge> RegistratorKFs::tryAndMatchRANSAC(const ptam::KeyFrame& kfa, const ptam::KeyFrame& kfb)
{
    boost::shared_ptr<ptam::Edge> edge;

    std::vector<cv::DMatch> matchesAB, matchesBA;
    std::vector<Observation> obsAB, obsBA;

    // match both ways
    matcher_->match(kfa.mpDescriptors, kfb.kpDescriptors, matchesAB);
    matcher_->match(kfb.mpDescriptors, kfa.kpDescriptors, matchesBA);

    std::cout << "mpts, matches sizes: " << kfa.mpDescriptors.rows << ", " << kfb.mpDescriptors.rows
            << ", " << matchesAB.size() << ", " << matchesBA.size() << std::endl;

    // RANSAC A->B:
    Sophus::SE3d relPoseAB;
    std::vector<int> inliers;
    std::cout << "doing RANSAC A to B" << std::endl;
    if (reg_3p_->solvePnP_RANSAC(kfa, kfb, matchesAB, relPoseAB, inliers, nMinInliers_)){
        reg_3p_->getObserv(kfa, kfb, matchesAB, inliers, obsAB);
    }
    else
        return edge;

    std::cout << "inliers obsAB: " << obsAB.size() << std::endl;

    // RANSAC B->A:
    Sophus::SE3d relPoseBA;
    std::vector<int> inliersBA;
    std::cout << "doing RANSAC B to A" << std::endl;
    if (reg_3p_->solvePnP_RANSAC(kfb, kfa, matchesBA, relPoseBA, inliersBA, nMinInliers_)){
        reg_3p_->getObserv(kfb, kfa, matchesBA, inliersBA, obsBA);
    }
    else
        return edge;

    std::cout << "inliers obsBA: " << obsBA.size() << std::endl;

    // compute angular error between both relative poses
    Sophus::SO3d err = relPoseAB.so3()*relPoseBA.so3();
    double theta;
    Sophus::SO3d::logAndTheta(err, &theta);
    double errdis = (relPoseAB.translation() - relPoseBA.translation()).norm();
    if (std::abs(theta) > maxErrAngle_ || errdis > maxErrDis_)
        return edge;

    // Both RANSAC poses agree. Refine
    reg_sim3_->solve(obsAB, obsBA);

    // change a, b order to make this edge consist with the definition for backward neighbours
    edge.reset(new ptam::Edge(kfb.id, kfa.id, ptam::EDGE_LOOP, reg_sim3_->aTb_se3().inverse()));
    return edge;
}

boost::shared_ptr<ptam::Edge> RegistratorKFs::tryAndMatch(const ptam::KeyFrame& kfa, const ptam::KeyFrame& kfb)
{
    boost::shared_ptr<ptam::Edge> edge;

    std::vector<cv::DMatch> matchesAB, matchesBA;
    std::vector<cv::DMatch> matchesABin, matchesBAin;
    std::vector<Observation> obsAB, obsBA;

    // match both ways
    matcher_->match(kfa.mpDescriptors, kfb.kpDescriptors, matchesAB);
    matcher_->match(kfb.mpDescriptors, kfa.kpDescriptors, matchesBA);

    std::cout << "mpts, matches sizes: " << kfa.mpDescriptors.rows << ", " << kfb.mpDescriptors.rows
            << ", " << matchesAB.size() << ", " << matchesBA.size() << std::endl;

    // RANSAC A->B:
    Sophus::SE3d relPoseAB = reg_3p_->solve(kfa, kfb, matchesAB);
    matchesABin = reg_3p_->getInliers(kfa, kfb, matchesAB, relPoseAB, threshPx_, obsAB);
    std::cout << "inliers: " << matchesABin.size() << std::endl;
    if (matchesABin.size() < matchesAB.size() * nMinInliers_)
         return edge;

    // RANSAC B->A:
    Sophus::SE3d relPoseBA = reg_3p_->solve(kfb, kfa, matchesBA);
    matchesBAin = reg_3p_->getInliers(kfb, kfa, matchesBA, relPoseBA, threshPx_, obsBA);
    std::cout << "inliers: " << matchesBAin.size() << std::endl;
    if (matchesBAin.size() < matchesBA.size() * nMinInliers_)
        return edge;

    // compute angular error between both relative poses
    Sophus::SO3d err = relPoseAB.so3()*relPoseBA.so3();
    double theta;
    Sophus::SO3d::logAndTheta(err, &theta);
    double errdis = (relPoseAB.translation() - relPoseBA.translation()).norm();
    if (std::abs(theta) > maxErrAngle_ || errdis > maxErrDis_)
        return edge;

    // Both RANSAC poses agree. Refine
    reg_sim3_->solve(obsAB, obsBA);

    // change a, b order to make this edge consist with the definition for backward neighbours
    edge.reset(new ptam::Edge(kfb.id, kfa.id, ptam::EDGE_LOOP, reg_sim3_->aTb_se3().inverse()));
    return edge;
}

bool RegistratorKFs::tryToRelocaliseRANSAC(const boost::shared_ptr<ptam::KeyFrame> kfa, const boost::shared_ptr<ptam::KeyFrame> kfb,
                                             Sophus::SE3d &result, double minInliers)
{
    std::vector<cv::DMatch> matchesAB;

    // match
    matcher_->match(kfa->mpDescriptors, kfb->kpDescriptors, matchesAB);

    std::cout << "Matches found for relocalization: " << matchesAB.size() << std::endl;
    // RANSAC A->B:
    Sophus::SE3d relPoseAB;
    std::vector<int> inliers;
    if (reg_3p_->solvePnP_RANSAC(*kfa, *kfb, matchesAB, relPoseAB, inliers, minInliers, threshPx_)){
        result = relPoseAB;
        return true;
    }
    else
        return false;
}

bool RegistratorKFs::tryToRelocalise(const boost::shared_ptr<ptam::KeyFrame> kfa, const boost::shared_ptr<ptam::KeyFrame> kfb,
                                             Sophus::SE3d &result, double minInliers)
{
    std::vector<cv::DMatch> matchesAB;
    std::vector<cv::DMatch> matchesABin;
    std::vector<Observation> obsAB;
    // match
    matcher_->match(kfa->mpDescriptors, kfb->kpDescriptors, matchesAB);

    std::cout << "Matches found for relocalization: " << matchesAB.size() << std::endl;
    // RANSAC A->B:
    Sophus::SE3d relPoseAB = reg_3p_->solve(*kfa, *kfb, matchesAB);
    std::cout << "RANSAC done for relocalization: " << std::endl;

    matchesABin = reg_3p_->getInliers(*kfa, *kfb, matchesAB, relPoseAB, threshPx_, obsAB);
    cout << "Inliers for reloc.: " << matchesABin.size() << endl;
    if (matchesABin.size() < matchesAB.size() * minInliers){
        std::cout << "Too few inliers from PnP" << std::endl;
        return false;
    }

    bool bReloc = reg_3p_->solvePnP(*kfa, *kfb, matchesABin, relPoseAB);
    std::cout << "relocalization refined by PnP " << std::endl;
    if (!bReloc) {
        std::cout << "No reloc. candidate got from PnP" << std::endl;
        return false;
    }

    double dis = relPoseAB.translation().norm();
    if (abs(dis) > 1.0){
        std::cout << "Too far away from the reference kf: "<< dis << std::endl;
        return false;
    }

    result = relPoseAB;
    return true;
}

// for detected large loop, we expect there's significant pose drift.
boost::shared_ptr<ptam::Edge> RegistratorKFs::tryAndMatchLargeLoopRANSAC(const ptam::KeyFrame& kfa, const ptam::KeyFrame& kfb)
{
    boost::shared_ptr<ptam::Edge> edge;
//    if (abs(kfa.id - kfb.id) < 20) // we need this
//        return edge;

    std::vector<cv::DMatch> matchesAB, matchesBA;

//    matcher_->match(kfa.mpDesc, kfb.kpDesc, matchesAB);
    matcher_->match(kfb.mpDescriptors, kfa.kpDescriptors, matchesBA);

    std::cout << "large loop inliers: " << matchesBA.size() << std::endl;
    if (matchesBA.size() < kfb.mpDescriptors.rows * nMinInliers_)
        return edge;

    // RANSAC B->A:
    Sophus::SE3d relPoseBA;
    std::vector<int> inliersBA;
    if (reg_3p_->solvePnP_RANSAC(kfb, kfa, matchesBA, relPoseBA, inliersBA, nMinInliers_)){
        // change a, b order to make this edge consist with the definition for backward neighbours
        edge.reset(new ptam::Edge(kfa.id, kfb.id, ptam::EDGE_LOOP, relPoseBA));
        return edge;
    }
    else
        return edge;

}

boost::shared_ptr<ptam::Edge> RegistratorKFs::tryAndMatchLargeLoop(const ptam::KeyFrame& kfa, const ptam::KeyFrame& kfb)
{
    boost::shared_ptr<ptam::Edge> edge;
//    if (abs(kfa.id - kfb.id) < 20) // we need this
//        return edge;

    std::vector<cv::DMatch> matchesAB, matchesBA;
    std::vector<cv::DMatch> matchesABin, matchesBAin;
    std::vector<Observation> obsAB, obsBA;

//    matcher_->match(kfa.mpDesc, kfb.kpDesc, matchesAB);
    matcher_->match(kfb.mpDescriptors, kfa.kpDescriptors, matchesBA);

    std::cout << "large loop inliers: " << matchesBA.size() << std::endl;

    // RANSAC B->A:
    Sophus::SE3d relPoseBA = reg_3p_->solve(kfb, kfa, matchesBA);
    matchesBAin = reg_3p_->getInliers(kfb, kfa, matchesBA, relPoseBA, threshPx_, obsBA);
    std::cout << "inliers: " << matchesBAin.size() << std::endl;
    if (matchesBAin.size() < matchesBA.size() * nMinInliers_)
        return edge;
    else{
        if (!reg_3p_->solvePnP(kfb, kfa, matchesBAin, relPoseBA))
            return edge;
        // change a, b order to make this edge consist with the definition for backward neighbours
        edge.reset(new ptam::Edge(kfa.id, kfb.id, ptam::EDGE_LOOP, relPoseBA));
        return edge;
    }
}
} // namespace


