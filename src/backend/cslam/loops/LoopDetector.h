#ifndef _CSLAM_LOOP_DETECTOR_
#define _CSLAM_LOOP_DETECTOR_

#include <boost/scoped_ptr.hpp>
#include <DBoW2/DBoW2.h>

#include <cs_geometry/Camera.h>
#include <ptam/KeyFrame.h>

#include <registration/Registrator5P.h>
#include <registration/RegistratorBAPTAM.h>

#include "DLoopDetector.h"
#include "TemplatedLoopDetector.h"


namespace cslam {

class LoopDetector {
public:
    LoopDetector(const cs_geom::Camera& cam, const std::string& vocFile);
    void reset();

    // Add newKF to database and use it to detect loops
    // return -1 if no loop was detected or the ID of the best match.
    int detectLoop(const ptam::KeyFrame& newKF);
protected:
    const cs_geom::Camera& cam_;
    boost::scoped_ptr<BriefVocabulary>   voc_;
    boost::scoped_ptr<BriefLoopDetector> detector_;

    BriefLoopDetector::Parameters        params_;

    std::map<int, int> kf2ld_, ld2kf_; // Convert between kf ID and LoopDetector ID
};

}

#endif /* _CSLAM_LOOP_DETECTOR_ */
