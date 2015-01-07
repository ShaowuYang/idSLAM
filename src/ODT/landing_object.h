/*
  manage the landing pad related frames, and traker
  images are handled with opencv mat image

*/
#ifndef LANDING_OBJECT_H
#define LANDING_OBJECT_H

#include <ros/ros.h>
#include <TooN/se3.h>
#include <TooN/wls.h>
#include <TooN/SVD.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "conversions_img.h"
#include "padtracker.h"
#include "polygon.h"

namespace ptam{
#define objLEVEL 3 //levels of image used for feature matching

class landing_object
{
public:
    landing_object();

    inline TooN::SE3<> GetCurrentObjectPose() const { return mse3object;}
    void reset();

    // @param:
    // descriptors_ref: descriptors of the ref image,
    // current_img: current keyframe
    //               the feature points from multiscaled (4 levels in PTAM) features,
    // object_corners: detected object (4 corners of the ref image in current keyframe)
    // H:       homography
    void pad_detection(cv::Mat descriptors_ref, cv::Mat current_img, std::vector<CVD::ImageRef> current_points,
                       cv::Mat &H, std::vector<CVD::ImageRef> &object_corners);
    void padfeature_detect(cv::Mat img, std::vector<cv::KeyPoint>& points);
    void padfeature_descript(cv::Mat& img, std::vector<cv::KeyPoint>& points, cv::Mat& descriptors);
    void currentfeature_descript(cv::Mat& img, std::vector<cv::KeyPoint>& points, cv::Mat& descriptors);
    void padfeature_match(cv::Mat descriptors1, cv::Mat descriptors2, std::vector<cv::DMatch>& matches);
    void padfeature_match(cv::Mat descriptors1, cv::Mat descriptors2, std::vector<std::vector<cv::DMatch> >& matches);

    void load_refcorners(std::vector<cv::KeyPoint> points, int level) { RefPoints[level] = points;}
    void load_curcorners(std::vector<cv::KeyPoint> points, int level) { CurrentPoints[level] = points;}
    void load_refimgs(cv::Mat img, int level) { RefImg[level] = img; }
    void load_curimgs(cv::Mat img, int level) { CurImg[level] = img; }
    void load_refdescriptors(cv::Mat descriptors, int level) { RefDescriptors[level] = descriptors; }
    void load_curdescriptors(cv::Mat descriptors, int level) { CurrentDesriptors[level] = descriptors; }
    void ini_allrefcorners(std::vector<cv::KeyPoint> points) { allRefPoints = points;}
    void ini_allcurcorners(std::vector<cv::KeyPoint> points) { allCurrentPoints = points;}
    void add_allrefcorners(cv::KeyPoint point) {
            allRefPoints.push_back(point);
    }
    void add_allcurcorners(cv::KeyPoint point) {
        allCurrentPoints.push_back(point);
    }
    void add_allORBrefcorners(std::vector<cv::KeyPoint> points) {
            allRefPoints = points;
    }
    void load_allrefdescriptors(int firstl) {// TODO: consider the case when i = 0, refpoints[0].size = 0
        allRefDescriptors = RefDescriptors[firstl];
        for (int i = firstl+1; i < objLEVEL; i ++){
            if (RefDescriptors[i].rows)
            allRefDescriptors.push_back(RefDescriptors[i]);
        }
//        cv::vconcat(allRefDescriptors, RefDescriptors[2], allRefDescriptors);
    }
    void load_allcurdescriptors() {
        allCurrentDesriptors = CurrentDesriptors[0];
        allCurrentDesriptors.push_back(CurrentDesriptors[1]);
//        for (int i = 1; i < objLEVEL-1; i ++){
//            if (CurrentDesriptors[i].rows)
//            allCurrentDesriptors.push_back(CurrentDesriptors[i]);
//        }
//        cv::vconcat(allCurrentDesriptors, CurrentDesriptors[1], allCurrentDesriptors);
    }
    void load_allORBrefdescriptors(cv::Mat descriptors) {
        allRefDescriptors = descriptors;
    }
    void load_matches(std::vector<cv::DMatch> matches, int level) { Matches[level] = matches; }
    void load_matches_ful(std::vector<cv::DMatch> matches) { Matches_Ful = matches; }

    cv::Mat& GetRefDescriptors(int level) { return RefDescriptors[level]; }
    cv::Mat& GetCurrentDescriptors(int level) { return CurrentDesriptors[level]; }
    cv::Mat& GetallRefDescriptors() { return allRefDescriptors; }
    cv::Mat& GetallCurrentDescriptors() { return allCurrentDesriptors; }
    std::vector<cv::DMatch>& GetMatches(int level) { return Matches[level];}
    std::vector<cv::DMatch>& GetMatches_ful() { return Matches_Ful;}
    cv::Mat GetRefimgs(int level) { return RefImg[level];}
    cv::Mat GetCurimgs(int level) { return CurImg[level];}
    std::vector<cv::KeyPoint> GetRefpoints(int level) {return RefPoints[level];}
    std::vector<cv::KeyPoint> GetCurpoints(int level) {return CurrentPoints[level];}
    std::vector<cv::KeyPoint> GetallRefpoints() {return allRefPoints;}
    std::vector<cv::KeyPoint> GetallCurpoints() {return allCurrentPoints;}

    // for landing pad tracker
    void makepadtrackerOrigin(CVD::Image<CVD::byte> &img, int level){
        mreferenceframe[level].makefromImg(img); }// the original reference landing pad template
    void makepadtrackerthis(CVD::Image<CVD::byte> &img){
        mpadtrackerthisframe = new PadTracker(img); }
    void makepadtrackerlast(CVD::Image<CVD::byte> &img){
        mpadtrackerlastframe = new PadTracker(img); }
    void makepadtrakerthisfromorigin(int level){
        mpadtrackerthisframe = &mreferenceframe[level]; }// copy the level original ref temp to thisframe
    void copypadtracker_this2last(){
        if (mpadtrackerlastframe)
            delete  mpadtrackerlastframe;
        mpadtrackerlastframe = mpadtrackerthisframe;
    }
    TooN::Matrix<3> padtrackermain(TooN::Matrix<3> h_);// main computation part of the padtracker

    bool pad_detected;
    ros::Time time_last_detect;
    bool isGood;
    mutable boost::shared_mutex mutex;

protected:
    cv::Mat RefImg[objLEVEL]; // useful only for debug?
    cv::Mat CurImg[objLEVEL];
    std::vector<cv::KeyPoint> RefPoints[objLEVEL];
    std::vector<cv::KeyPoint> CurrentPoints[objLEVEL];
    cv::Mat RefDescriptors[objLEVEL];
    cv::Mat CurrentDesriptors[objLEVEL];
    std::vector<cv::DMatch> Matches[objLEVEL*2];// matches found for each ref img levels
    std::vector<cv::KeyPoint> allRefPoints;
    std::vector<cv::KeyPoint> allCurrentPoints;
    cv::Mat allRefDescriptors;
    cv::Mat allCurrentDesriptors;
    std::vector<cv::DMatch> Matches_Ful; // the consistant match of all levels.//after project matches in different level to levle 0
    TooN::SE3<> mse3object;//landing object pose in the map

    cv::Ptr<cv::FeatureDetector> featuredetector_ref;
    cv::Ptr<cv::DescriptorExtractor> descripterextractor_ref;
    cv::Ptr<cv::FeatureDetector> featuredetector;
    cv::Ptr<cv::DescriptorExtractor> descripterextractor;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // for landing pad tracking
    PadTracker mreferenceframe[objLEVEL];// only for the first reference template, which need to store all objlevels
    PadTracker *mpadtrackerthisframe; // padtracker represent the reference template. Input the target image for tracking
    PadTracker *mpadtrackerlastframe;

};
}
#endif // LANDING_OBJECT_H
