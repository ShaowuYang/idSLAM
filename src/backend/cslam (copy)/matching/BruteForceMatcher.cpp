#include "BruteForceMatcher.h"

namespace cslam {

BruteForceMatcher::BruteForceMatcher(const cs_geom::Camera& cam)
{
    matcher_.reset(new cv::BFMatcher(cv::NORM_HAMMING, true));
    reg_5p_.reset(new Registrator5P(cam));
    reg_ba_.reset(new RegistratorBAPTAM(cam));
}

boost::shared_ptr<Edge> BruteForceMatcher::tryAndMatch(const Keyframe& kf1, const Keyframe& kf2)
{

    boost::shared_ptr<Edge> edge;
    return edge;

    std::vector<cv::DMatch> matches;
    matcher_->match(kf1.kpDesc, kf2.kpDesc, matches);

    cv::Mat debugImage;
    cv::drawMatches(kf1.rgbImage, kf1.keypoints, kf2.rgbImage, kf2.keypoints, matches, debugImage);
    cv::imshow("matches", debugImage);
    cv::waitKey(0);

    reg_5p_->solve(
                kf1.keypoints, kf1.kpDepth,
                kf2.keypoints, kf2.kpDepth,
                matches);

    cv::drawMatches(kf1.rgbImage, kf1.keypoints, kf2.rgbImage, kf2.keypoints, reg_5p_->inliers(), debugImage);
    cv::imshow("matches", debugImage);
    cv::waitKey(0);

    reg_ba_->solve(
                kf1.keypoints, kf1.kpDepth,
                kf2.keypoints, kf2.kpDepth,
                reg_5p_->inliers(), reg_5p_->relPose0T1());

    cv::drawMatches(kf1.rgbImage, kf1.keypoints, kf2.rgbImage, kf2.keypoints, reg_ba_->inliers(), debugImage);
    cv::imshow("matches", debugImage);
    cv::waitKey(0);
}

}
