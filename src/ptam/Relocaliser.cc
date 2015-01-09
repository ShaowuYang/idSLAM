// Copyright 2008 Isis Innovation Limited
#include "Relocaliser.h"
#include "SmallBlurryImage.h"
#include "KeyFrame.h"
#include <cvd/utility.h>
#include <gvars3/instances.h>

using namespace CVD;
using namespace GVars3;
using namespace std;
using namespace TooN;
using namespace ptam;

Relocaliser::Relocaliser(Map &map)
  : mMap(map), mCamera(CameraModel::CreateCamera()),
    mCameraSec(CameraModel::CreateCamera(1))
{
};

SE3<> Relocaliser::BestPose()
{
  return mse3Best;
}

bool Relocaliser::AttemptRecovery(KeyFrame &kCurrent)
{
    double dScore;
  // Ensure the incoming frame has a SmallBlurryImage attached
  kCurrent.SBI.MakeFromKF(kCurrent);
  
  // Find the best ZMSSD match from all keyframes in map
  ScoreKFs(kCurrent);

  if (!kCurrent.nSourceCamera){
      // And estimate a camera rotation from a 3DOF image alignment
      pair<SE2<>, double> result_pair = kCurrent.SBI.IteratePosRelToTarget(mMap.vpKeyFrames[mnBest]->SBI, 6);
      mse2 = result_pair.first;
      dScore =result_pair.second;

      SE3<> se3KeyFramePos = mMap.vpKeyFrames[mnBest]->se3CfromW;
      // SE3fromSE2 will set the corret image size for the camera, so no need to call CameraModel::SetImageSize()
      mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mCamera.get()) * se3KeyFramePos;
  }else
  {
      pair<SE2<>, double> result_pair = kCurrent.SBI.IteratePosRelToTarget(mMap.vpKeyFramessec[kCurrent.nSourceCamera - 1][mnBest2]->SBI, 6);
      mse2sec = result_pair.first;
      dScore =result_pair.second;
      cout << "Make an attempt of recovering pose." << endl;

      SE3<> se3KeyFramePos = mMap.vpKeyFramessec[kCurrent.nSourceCamera - 1][mnBest2]->se3CfromW;
      // SE3fromSE2 will set the corret image size for the camera, so no need to call CameraModel::SetImageSize()
      mse3Best = SmallBlurryImage::SE3fromSE2(mse2sec, mCameraSec[kCurrent.nSourceCamera - 1].get()) * se3KeyFramePos;
  }
  if(dScore < GV2.GetDouble("Reloc2.MaxScore", 9e6, SILENT))
    return true;
  else 
    return false;
};

bool Relocaliser::AttemptRecoveryDual(KeyFrame &kCurrent, KeyFrame &kCurrentsec)
{
  // Ensure the incoming frame has a SmallBlurryImage attached
  kCurrent.SBI.MakeFromKF(kCurrent);

  // Find the best ZMSSD match from all keyframes in map
  ScoreKFs(kCurrent);

  // And estimate a camera rotation from a 3DOF image alignment
  pair<SE2<>, double> result_pair = kCurrent.SBI.IteratePosRelToTarget(mMap.vpKeyFrames[mnBest]->SBI, 6);
  mse2 = result_pair.first;
  double dScore1 =result_pair.second;

  // do the same for the second cam
  kCurrentsec.SBI.MakeFromKF(kCurrentsec);
  ScoreKFs(kCurrentsec);
  pair<SE2<>, double> result_pair2 = kCurrentsec.SBI.IteratePosRelToTarget(mMap.vpKeyFramessec[mnBest2]->SBI, 6);
  mse2sec = result_pair2.first;
  double dScore2 =result_pair2.second;

  double dScore = min(dScore1, dScore2);
  if(dScore > GV2.GetDouble("Reloc2.MaxScore", 9e6, SILENT))
    return false;

  if (dScore1 < dScore2){
      SE3<> se3KeyFramePos = mMap.vpKeyFrames[mnBest]->se3CfromW;
      // SE3fromSE2 will set the corret image size for the camera, so no need to call CameraModel::SetImageSize()
      mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mCamera.get()) * se3KeyFramePos;
  }else
  {
      SE3<> se3KeyFramePos = mMap.vpKeyFramessec[mnBest2]->se3CfromW;
      // SE3fromSE2 will set the corret image size for the camera, so no need to call CameraModel::SetImageSize()
      mse3Best = SmallBlurryImage::SE3fromSE2(mse2sec, mCameraSec.get()) * se3KeyFramePos;
  }

  return true;
};
// Compare current KF to all KFs stored in map by
// Zero-mean SSD
void Relocaliser::ScoreKFs(KeyFrame &kCurrent)
{
  if (!kCurrent.nSourceCamera){
      mdBestScore = 99999999999999.9;
      mnBest = -1;
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
      {
          double dSSD = kCurrent.SBI.ZMSSD(mMap.vpKeyFrames[i]->SBI);
          if(dSSD < mdBestScore)
          {
              mdBestScore = dSSD;
              mnBest = i;
          }
      }
  } else
  {
      int ncamIndex = kCurrent.nSourceCamera - 1;
      mdBestScore2 = 99999999999999.9;
      mnBest2 = -1;

      for(unsigned int i=mMap.vpKeyFramessec[ncamIndex].size()-1; i>=max((int)mMap.vpKeyFramessec[ncamIndex].size()-4, 0); i--)
        {
          // yang, only relocalise around recent 4 kfs
            double dSSD = kCurrent.SBI.ZMSSD(mMap.vpKeyFramessec[ncamIndex][i]->SBI);
            if(dSSD < mdBestScore2)
            {
                mdBestScore2 = dSSD;
                mnBest2 = i;
            }
        }
    }
}

