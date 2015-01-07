/*

 landing pad tracker locate here. Mainly refer to ESM method implemented by libcvd.
 also refer to code of ptam/smallblurryimage by Klein.
 images are handled in cvd::image since we use libcvd for this.

  */

#ifndef PADTRACKER_H
#define PADTRACKER_H

#include <cvd/esm.h>

namespace ptam{
class KeyFrame;
class CameraModel;

class PadTracker
{
public:
    PadTracker();

    // create the pad tracker, which contain the reference template information
    PadTracker(CVD::Image<CVD::byte> &img);
    void makefromImg(CVD::Image<CVD::byte> &img);

    // make the Jacobians of the image
    void MakeJacs();

    // the main iterate process for tracking, calculating the homography, now using ESM
    TooN::Matrix<3> trackhomography_ESM(TooN::Matrix<3> h_, PadTracker *other, int nIterations = 10);

    TooN::Matrix<3> gethomography(){ return mHomography; }

protected:
    CVD::Image<CVD::byte> mimSmall;// the original image copy
    CVD::Image<float> mimTemplate;// used for tracking, after some basic pre-procession
    CVD::Image<TooN::Vector<2> > mimImageJacs;
    bool mbMadeJacs;
    CVD::ImageRef mirSize;// size of this image template to be tracked

    CVD::ESMResult mTrackingResult;// some result of the optimisition of ESM
    CVD::Homography<8> mTransform;// tracking result, the transform, which contain the homography...
                                  // when serve as input of ESM, it is also the initial homography,
                                  // which should be as accurate as possible
    CVD::StaticAppearance mAppearance;
    TooN::Matrix<3> mHomography;// tracking result, the homography, this is what we most needed
};
}
#endif // PADTRACKER_H
