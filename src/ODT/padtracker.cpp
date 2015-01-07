#include "padtracker.h"

using namespace std;
using namespace CVD;
using namespace ptam;
PadTracker::PadTracker()
{
    mHomography = TooN::Identity;
    mbMadeJacs = false;
}

PadTracker::PadTracker(Image<CVD::byte> &img)
{
    mHomography = TooN::Identity;

    makefromImg(img);
    mbMadeJacs = false;
}

void PadTracker::makefromImg(Image<CVD::byte> &img)
{
    mirSize = img.size();
    mimSmall.resize(mirSize);
    mimTemplate.resize(mirSize);

    mimSmall = img;
    ImageRef ir;
    unsigned int nSum = 0;
    do
      nSum += mimSmall[ir];
    while(ir.next(mirSize));

    float fMean = ((float) nSum) / mirSize.area();

    ir.home();
    do
      mimTemplate[ir] = mimSmall[ir] - fMean;
    while(ir.next(mirSize));
}

void PadTracker::MakeJacs()
{
    mimImageJacs.resize(mirSize);

    gradient(mimTemplate, mimImageJacs);
    mbMadeJacs = true;
}

TooN::Matrix<3> PadTracker::trackhomography_ESM(TooN::Matrix<3> h_, PadTracker *other, int nIterations)
{
    MakeJacs();
    CVD::Homography<8> mT_(h_);
    mTransform = mT_;
    mTrackingResult = Internal::esm_opt(mTransform, mAppearance, mimTemplate, mimImageJacs,
                                             other->mimTemplate, nIterations, 1e-8, 1.0);
    mHomography = mTransform.get_matrix();

    return mHomography;
}
