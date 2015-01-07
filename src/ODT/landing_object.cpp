#include "landing_object.h"

#include "MyOrb.h"

#define Orbsize 15
#define OrbOctav 3
using namespace ptam;
using namespace std;
landing_object::landing_object()
{
    featuredetector_ref = new cv::ORB(500, 1.2, OrbOctav, Orbsize, 0, 2,
                                        cv::ORB::HARRIS_SCORE, Orbsize);
    featuredetector = new cv::FastFeatureDetector(40);
//    descripterextractor = new cv::FREAK(true, true, 15, 1);
//    descripterextractor_ref = new cv::FREAK(true, true, 15, 1);
//    descripterextractor_ref = new cv::ORB(300, 2, OrbOctav, Orbsize, 0, 2, cv::ORB::HARRIS_SCORE, Orbsize);
//    descripterextractor = new cv::ORB(600, 2, OrbOctav, Orbsize, 0, 2, cv::ORB::HARRIS_SCORE, Orbsize);

    descripterextractor_ref = new cv::MyORB(500, 1.2, OrbOctav, Orbsize, 0, 2,
                                            cv::ORB::HARRIS_SCORE, Orbsize);
    descripterextractor = new cv::MyORB(600, 2, OrbOctav, Orbsize, 0, 2,
                                        cv::ORB::HARRIS_SCORE, Orbsize);

    matcher = new cv::BFMatcher(cv::NORM_HAMMING, false);
    std::cout << "DESCRIPTOR INIED." << std::endl;

    mpadtrackerthisframe = NULL;
    mpadtrackerlastframe = NULL;
}

void landing_object::reset()
{
    pad_detected = false;
}

void landing_object::pad_detection(cv::Mat descriptors_ref, cv::Mat current_img, vector<CVD::ImageRef> current_points,
                                   cv::Mat &H, vector<CVD::ImageRef> &object_corners)
{

}

void landing_object::padfeature_detect(cv::Mat img, vector<cv::KeyPoint>& points)
{
    featuredetector_ref->detect(img, points);
}

void landing_object::padfeature_descript(cv::Mat& img, std::vector<cv::KeyPoint>& points, cv::Mat& descriptors)
{
//    std::cout  << "Original points num: "<< points.size() << std::endl;
    descripterextractor_ref->compute(img, points, descriptors);
//    std::cout << "Points and descriptors num: " << points.size() << " " << descriptors.rows << std::endl;

}

void landing_object::currentfeature_descript(cv::Mat& img, std::vector<cv::KeyPoint>& points, cv::Mat& descriptors)
{
//    std::cout  << "Original points num: "<< points.size() << std::endl;
    descripterextractor->compute(img, points, descriptors);
//    std::cout << "Points and descriptors num: " << points.size() << " " << descriptors.rows << std::endl;

}


void landing_object::padfeature_match(cv::Mat descriptors1, cv::Mat descriptors2, vector<cv::DMatch>& matches)
{
    matcher->match(descriptors1, descriptors2, matches);
//    std::cout << "Descriptors and matches num: " << descriptors1.rows << " " << matches.size() << std::endl;
}

void landing_object::padfeature_match(cv::Mat descriptors1, cv::Mat descriptors2, vector<vector<cv::DMatch> >& matches)
{
    int knn = 2;
    matcher->knnMatch(descriptors1, descriptors2, matches, knn);
//    std::cout << "Descriptors and matches num: " << descriptors1.rows << " " << matches.size() << std::endl;
}

// compute the homography of the padtrackerthis with respect to padtrackerlast
TooN::Matrix<3> landing_object::padtrackermain(TooN::Matrix<3> h_)
{
    return mpadtrackerthisframe->trackhomography_ESM(h_, mpadtrackerlastframe, 20);
}

