// -*- c++ -*- 
// Copyright 2008 Isis Innovation Limited
//
// SmallBlurryImage-based relocaliser
// 
// Each KF stores a small, blurred version of itself;
// Just compare a small, blurred version of the input frame to all the KFs,
// choose the closest match, and then estimate a camera rotation by direct image
// minimisation.

#ifndef __RELOCALISER_H
#define __RELOCALISER_H
#include <TooN/se2.h>
#include <memory>
#include "CameraModel.h"
#include "SmallBlurryImage.h"
#include "Map.h"

namespace ptam{
class Relocaliser
{
public:
  Relocaliser(Map &map);
  bool AttemptRecovery(KeyFrame &k);
  TooN::SE3<> BestPose();
  bool AttemptRecoveryDual(KeyFrame &k, KeyFrame &k2);

protected:
  void ScoreKFs(KeyFrame &kCurrentF);
  Map &mMap;
  int mnBest;
  double mdBestScore;
  int mnBest2;
  double mdBestScore2;
  TooN::SE2<> mse2;
  TooN::SE2<> mse2sec;
  TooN::SE3<> mse3Best;
  std::auto_ptr<CameraModel> mCamera;
  std::auto_ptr<CameraModel> mCameraSec;
};
} // namespace

#endif









