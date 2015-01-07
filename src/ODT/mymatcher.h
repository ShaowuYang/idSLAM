#ifndef MYMATCHER_H
#define MYMATCHER_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/legacy/legacy.hpp"

using namespace cv;

class Mymatcher
{
public:
    Mymatcher();

    cv::Ptr<cv::DescriptorMatcher> matcher;

    void featurematch(cv::Mat descriptors1, cv::Mat descriptors2, vector<vector<cv::DMatch> >& matches);
};

#endif // MYMATCHER_H
