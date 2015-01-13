#include "BruteForceMatcher.h"
#include <cvd/image_io.h>

namespace backend {

BruteForceMatcher::BruteForceMatcher(const cs_geom::Camera& cam)
{
    matcher_.reset(new cv::BFMatcher(cv::NORM_HAMMING, true));
    reg_5p_.reset(new Registrator5P(cam));
    reg_ba_.reset(new RegistratorBAPTAM(cam));
}

boost::shared_ptr<ptam::Edge> BruteForceMatcher::tryAndMatch(const ptam::KeyFrame& kf1, const ptam::KeyFrame& kf2)
{

    boost::shared_ptr<ptam::Edge> edge;
    return edge;

    std::vector<cv::DMatch> matches;
    matcher_->match(kf1.kpDescriptors, kf2.kpDescriptors, matches);

    ptam::Level lev = kf1.aLevels[0];
    CVD::ImageRef irsize = lev.im.size();
    cv::Mat kf1im = cv::Mat(irsize[1], irsize[0], CV_8UC1,
                        (void*) lev.im.data(), lev.im.row_stride()).clone();
    lev = kf2.aLevels[0];
    cv::Mat kf2im = cv::Mat(irsize[1], irsize[0], CV_8UC1,
                        (void*) lev.im.data(), lev.im.row_stride()).clone();

    cv::Mat debugImage;
    cv::drawMatches(kf1im, kf1.keypoints, kf2im, kf2.keypoints, matches, debugImage);
    cv::imshow("matches", debugImage);
    cv::waitKey(0);

    reg_5p_->solve(
                kf1.keypoints, kf1.kpDepth,
                kf2.keypoints, kf2.kpDepth,
                matches);

    cv::drawMatches(kf1im, kf1.keypoints, kf2im, kf2.keypoints, reg_5p_->inliers(), debugImage);
    cv::imshow("matches", debugImage);
    cv::waitKey(0);

    reg_ba_->solve(
                kf1.keypoints, kf1.kpDepth,
                kf2.keypoints, kf2.kpDepth,
                reg_5p_->inliers(), reg_5p_->relPose0T1());

    cv::drawMatches(kf1im, kf1.keypoints, kf2im, kf2.keypoints, reg_ba_->inliers(), debugImage);
    cv::imshow("matches", debugImage);
    cv::waitKey(0);
}

}
