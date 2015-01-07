#include "LoopDetector.h"

#include <fstream>
#include <boost/lexical_cast.hpp>

using namespace boost;
using namespace cslam;
using namespace cs_geom;
using namespace std;

LoopDetector::LoopDetector(const Camera& cam, const std::string& vocFile) : cam_(cam)
{
    params_ = BriefLoopDetector::Parameters(cam.width(), cam.height());
    params_.use_nss = true; // use normalized similarity score instead of raw score
    params_.alpha = 0.0; // nss threshold
    params_.k = 0; // a loop must be consistent with 1 previous matches
    params_.geom_check = DLoopDetector::GEOM_NONE; //GEOM_DI;// use direct index for geometrical checking
    params_.di_levels = 2; // use two direct index levels

    cout << "LoopDetector: loading vocabulary: " << vocFile << endl;
    voc_.reset(new BriefVocabulary(vocFile));

    reset();
}

void LoopDetector::reset()
{
    cout << "LoopDetector: creating detector" << endl;
    detector_.reset(new BriefLoopDetector(*voc_, params_));
    detector_->allocate(300);

    kf2ld_.clear();
    ld2kf_.clear();
}

int LoopDetector::detectLoop(const ptam::KeyFrame& kf)
{
    vector<FBrief::TDescriptor> descriptors(kf.kpDescriptors.rows);

    for (uint i = 0; i < kf.kpDescriptors.rows; i++) {
        FBrief::fromMat8UC(descriptors[i], kf.kpDescriptors.row(i));
    }

    DLoopDetector::DetectionResult result;
    detector_->detectLoop(kf.keypoints, descriptors, result);
    std::cout << "loop detector: " << result.status << std::endl;

    kf2ld_[kf.id] = result.query;
    ld2kf_[result.query] = kf.id;

    if (result.detection()) {
        return ld2kf_[result.match];
    } else {
        return -1;
    }
}
