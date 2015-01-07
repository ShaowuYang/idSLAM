#include "Keyframe.h"

#include <fstream>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>

using namespace boost;
using namespace cslam;
using namespace cv;
using namespace std;

void Keyframe::finalizeKeyframe()
{
    assert(keypoints.empty());

    boost::scoped_ptr<cv::DescriptorExtractor> extractor(new cv::BriefDescriptorExtractor(32));

    // Extract descriptors of map points
    // Intermediate data structure: map points per level
    std::vector<std::vector<cv::KeyPoint> >mpKpts(levels.size());
    for (uint i = 0; i < mapPoints.size(); i++) {
        int l = mapPoints[i].level;
        cv::KeyPoint kp(cv::Point2f(mapPoints[i].pi.x, mapPoints[i].pi.y),
                        (1 << l)*10,
                        -1, 0,
                        l);

        kp.class_id = i; // store index of original map point
        mpKpts[l].push_back(kp);
    }

    // compute descriptors per level
    std::vector<MapPoint> mpNew;
    mpKeypoints.clear();
    for (uint l = 0; l < levels.size(); l++) {
        cv::Mat levDesc;
        // Warning: this modifies kpts (deletes keypoints for which it cannot compute a descriptor)
        extractor->compute(levels[l].image, mpKpts[l], levDesc);

        assert(mpKpts[l].size() == levDesc.rows);

        mpDesc.push_back(levDesc);

        // Keep only map points with good descriptors:
        for (int i = 0; i < mpKpts[l].size(); i++) {
            mpKeypoints.push_back(mpKpts[l][i]);
            mpNew.push_back(mapPoints[mpKpts[l][i].class_id]);
        }
        assert(mpDesc.rows == mpKeypoints.size());
    }
    mapPoints = mpNew;

    // Extract descriptors of corners
    for (uint l = 0; l < levels.size(); l++) {
        int scaleFactor = (1 << l);

        std::vector<cv::KeyPoint> kpts;
        kpts.resize(levels[l].corners.size());

        for (uint k = 0; k < kpts.size(); k++) {
            kpts[k].pt.x = levels[l].corners[k].x*scaleFactor;
            kpts[k].pt.y = levels[l].corners[k].y*scaleFactor;
            kpts[k].octave = l;
            kpts[k].angle = -1;
            kpts[k].size = (1 << l)*10; // setting this is required for DescriptorExtractors to work.

            // Abuse response field, but we have to keep this keypoint's depth somewhere
//            kpts[k].response = levels[l].cornerDepth[k];
        }

        cv::Mat levDesc;
        // Warning: this modifies kpts (deletes keypoints for which it cannot compute a descriptor)
        extractor->compute(levels[l].image, kpts, levDesc);

        assert(levDesc.rows == kpts.size());

        // concatenate all levels
        kpDesc.push_back(levDesc);
        keypoints.insert( keypoints.end(), kpts.begin(), kpts.end() );
    }

    // extract depth again
//    kpDepth.resize(keypoints.size());
//    for (unsigned int i = 0; i < keypoints.size(); i++) {
//        kpDepth[i] = keypoints[i].response;
//    }
}

void Keyframe::saveKeyframe() const
{
    saveKeyframe(string("kf") + lexical_cast<string>(id));
}

void Keyframe::saveKeyframe(const std::string& name) const
{
    imwrite(name + string(".png"), levels[0].image);
    imwrite(name + string("depth.png"), depthImage);

    ofstream kpout((name +string("keypoints.txt")).c_str());
    for (unsigned int i = 0; i < keypoints.size(); i++) {
        kpout << keypoints[i].pt.x << " " << keypoints[i].pt.y << " " << kpDepth[i] << endl;
    }

    ofstream cornersout((name +string("corners.txt")).c_str());
    for (uint l = 0; l < levels.size(); l++) {
        int scaleFactor = (1 << l);

        for (uint k = 0; k < levels[l].corners.size(); k++) {
            cornersout << levels[l].corners[k].x*scaleFactor << " "
                       << levels[l].corners[k].y*scaleFactor << " "
                       << levels[l].cornerDepth[k] << endl;
        }
    }
}
